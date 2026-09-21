function u = mpc_controller_block(y, r, u_last, p)
%#codegen
% Multirate fixed-model MPC for the hydropower active-power loop.
%
% The plant runs at 0.01 s. The ARX model and optimizer run every fifth
% call (0.05 s); the command is held between optimizer updates.
%
%   y(k+1) = a1*y(k) + a2*y(k-1) + b1*u(k) + b2*u(k-1) + bias
%
% p = [a1, a2, b1, b2, bias, output_weight, move_weight,
%      du_limit, u_min, u_max]

persistent y_previous
persistent held_input
persistent move_plan
persistent prediction_free
persistent prediction_moves
persistent hessian
persistent gradient_step
persistent sample_counter
persistent initialized

prediction_horizon = 40;
control_horizon = 4;
iterations = 20;
execution_divider = 5;

if isempty(initialized)
    a1 = p(1);
    a2 = p(2);
    b1 = p(3);
    b2 = p(4);
    bias = p(5);
    output_weight = p(6);
    move_weight = p(7);

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

    hessian = 2.0 * ( ...
        output_weight * (prediction_moves' * prediction_moves) + ...
        move_weight * eye(control_horizon));
    gradient_step = 0.8 / max(max(sum(abs(hessian), 2)), 1e-9);

    y_previous = y;
    held_input = u_last;
    move_plan = zeros(4, 1);
    sample_counter = 0;
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
u_min = p(9);
u_max = p(10);

state = [y; y_previous; held_input; 1.0];
reference = r * ones(prediction_horizon, 1);
free_error = prediction_free * state - reference;
gradient_offset = 2.0 * output_weight * ...
    (prediction_moves' * free_error);

for iteration = 1:iterations
    move_plan = move_plan - gradient_step * ...
        (hessian * move_plan + gradient_offset);

    cumulative_input = held_input;

    for move_index = 1:control_horizon
        move_plan(move_index) = max( ...
            -du_limit, min(move_plan(move_index), du_limit));
        proposed_input = cumulative_input + move_plan(move_index);

        if proposed_input > u_max
            move_plan(move_index) = u_max - cumulative_input;
        elseif proposed_input < u_min
            move_plan(move_index) = u_min - cumulative_input;
        end

        cumulative_input = cumulative_input + move_plan(move_index);
    end
end

held_input = max(u_min, min(held_input + move_plan(1), u_max));
u = held_input;
y_previous = y;
move_plan(1:end-1) = move_plan(2:end);
move_plan(end) = 0.0;
end
