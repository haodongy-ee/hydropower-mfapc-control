function u = mpc_23_controller_block(y, r, u_last, p)
%#codegen
% Phase-2.3 fixed-model MPC. Plant: 0.01 s; optimizer: 0.05 s.
% p(1:14) are the frozen phase-1 parameters. Added settings are:
% [measurement_alpha, disturbance_alpha, output_min, output_max,
%  soft_output_weight, antiwindup_decay].

persistent y_previous held_input move_plan prediction_free prediction_moves
persistent hessian gradient_step sample_counter filtered_reference
persistent filtered_measurement integral_trim previous_reference gate_counter
persistent disturbance_estimate predicted_next_output initialized
persistent filter_fast_counter

prediction_horizon = 40;
control_horizon = 4;
iterations = 20;
execution_divider = 5;

if isempty(initialized)
    a1 = p(1); a2 = p(2); b1 = p(3); b2 = p(4); bias = p(5);
    output_weight = p(6); move_weight = p(7);
    state_matrix = [ ...
        a1, a2, b1 + b2, bias; ...
        1.0, 0.0, 0.0, 0.0; ...
        0.0, 0.0, 1.0, 0.0; ...
        0.0, 0.0, 0.0, 1.0];
    input_matrix = [b1; 0.0; 1.0; 0.0];
    output_matrix = [1.0, 0.0, 0.0, 0.0];
    prediction_free = zeros(40, 4);
    prediction_moves = zeros(40, 4);
    state_power = eye(4);
    impulse_response = zeros(40, 1);
    for prediction_step = 1:prediction_horizon
        state_power = state_power * state_matrix;
        prediction_free(prediction_step, :) = output_matrix * state_power;
        if prediction_step == 1
            impulse_response(prediction_step) = output_matrix * input_matrix;
        else
            impulse_response(prediction_step) = ...
                prediction_free(prediction_step - 1, :) * input_matrix;
        end
    end
    for prediction_step = 1:prediction_horizon
        for move_index = 1:control_horizon
            response_index = prediction_step - move_index + 1;
            if response_index >= 1
                prediction_moves(prediction_step, move_index) = ...
                    impulse_response(response_index);
            end
        end
    end
    hessian = 2.0 * (output_weight * ...
        (prediction_moves' * prediction_moves) + ...
        move_weight * eye(control_horizon));
    % Include the worst-case curvature of the squared soft-output penalty.
    gradient_step = 0.8 / max(max(sum(abs(hessian), 2)) + ...
        2.0 * p(19) * max(max(sum(abs(prediction_moves' * ...
        prediction_moves), 2)), 0), 1e-9);

    y_previous = y;
    held_input = u_last;
    move_plan = zeros(4, 1);
    sample_counter = 0;
    filtered_reference = r;
    filtered_measurement = y;
    integral_trim = 0.0;
    previous_reference = r;
    gate_counter = 0;
    disturbance_estimate = 0.0;
    predicted_next_output = y;
    filter_fast_counter = 0;
    initialized = true;
end

if sample_counter > 0
    sample_counter = sample_counter - 1;
    u = held_input;
    return;
end
sample_counter = execution_divider - 1;

output_weight = p(6);
du_limit = p(8);
u_min = p(9); u_max = p(10);
integral_gain = p(11); integral_limit = p(12);
integration_gate = p(13); reference_filter_alpha = p(14);
measurement_alpha = p(15); disturbance_alpha = p(16);
output_min = p(17); output_max = p(18);
soft_output_weight = p(19); antiwindup_decay = p(20);

if abs(r - previous_reference) > 1e-8
    integral_trim = 0.0;
    gate_counter = 0;
    filter_fast_counter = 80;
end
previous_reference = r;
% Follow set-point changes for four seconds at a fast bandwidth. At steady
% operation use the stronger low-pass setting to suppress sensor noise.
if filter_fast_counter > 0 || ...
        abs(r - filtered_measurement) > 1.5 * integration_gate
    active_measurement_alpha = 0.8;
else
    active_measurement_alpha = measurement_alpha;
end
filter_fast_counter = max(filter_fast_counter - 1, 0);
% Filter only the controller measurement; the plant output and evaluation
% signals remain unfiltered. Filter state is retained between MPC updates.
filtered_measurement = filtered_measurement + ...
    active_measurement_alpha * (y - filtered_measurement);
filtered_reference = filtered_reference + ...
    reference_filter_alpha * (r - filtered_reference);

% Innovation from the preceding one-step prediction estimates a constant
% additive output disturbance. Bound it to avoid reacting to large steps.
innovation = filtered_measurement - predicted_next_output;
disturbance_estimate = disturbance_estimate + ...
    disturbance_alpha * innovation;
disturbance_estimate = max(-integral_limit, ...
    min(disturbance_estimate, integral_limit));

tracking_error = r - filtered_measurement;
pushing_high = held_input >= u_max - du_limit && tracking_error > 0;
pushing_low = held_input <= u_min + du_limit && tracking_error < 0;
pushing_output_high = filtered_measurement >= output_max && ...
    tracking_error > 0;
pushing_output_low = filtered_measurement <= output_min && ...
    tracking_error < 0;
if pushing_high || pushing_low || pushing_output_high || pushing_output_low
    % Unwind the accumulated target correction when a limit prevents the
    % requested action. In the interior, preserve phase-1 gated integration.
    integral_trim = (1.0 - antiwindup_decay) * integral_trim;
    gate_counter = 0;
elseif abs(tracking_error) <= integration_gate
    gate_counter = min(gate_counter + 1, 11);
    if gate_counter > 10
        integral_trim = integral_trim + integral_gain * tracking_error;
        integral_trim = max(-integral_limit, ...
            min(integral_trim, integral_limit));
    end
else
    gate_counter = 0;
end

state = [filtered_measurement; y_previous; held_input; 1.0];
corrected_reference = filtered_reference + integral_trim;
reference = corrected_reference * ones(prediction_horizon, 1);
free_output = prediction_free * state + disturbance_estimate;
free_error = free_output - reference;
gradient_offset = 2.0 * output_weight * ...
    (prediction_moves' * free_error);

for iteration = 1:iterations
    predicted_output = free_output + prediction_moves * move_plan;
    soft_violation = max(predicted_output - output_max, 0.0) + ...
        min(predicted_output - output_min, 0.0);
    soft_gradient = 2.0 * soft_output_weight * ...
        (prediction_moves' * soft_violation);
    move_plan = move_plan - gradient_step * ...
        (hessian * move_plan + gradient_offset + soft_gradient);
    cumulative_input = held_input;
    for move_index = 1:control_horizon
        move_plan(move_index) = max(-du_limit, ...
            min(move_plan(move_index), du_limit));
        proposed_input = cumulative_input + move_plan(move_index);
        if proposed_input > u_max
            move_plan(move_index) = u_max - cumulative_input;
        elseif proposed_input < u_min
            move_plan(move_index) = u_min - cumulative_input;
        end
        cumulative_input = cumulative_input + move_plan(move_index);
    end
end

predicted_next_output = free_output(1) + ...
    prediction_moves(1, :) * move_plan;
held_input = max(u_min, min(held_input + move_plan(1), u_max));
u = held_input;
y_previous = filtered_measurement;
move_plan(1:end-1) = move_plan(2:end);
move_plan(end) = 0.0;
end
