function buts_mid_pos = buttons_mid_pos_rel(rows_num_buttons, rows_button_keys, x_mid_pos, y_mid_pos, keyboard_SE3)
    if size(x_mid_pos) ~= size(y_mid_pos)
        error("Should be the same amount of X and Y coordinates given.")
    end
    
    rows_num = size(x_mid_pos,1);
    cols_num = size(x_mid_pos,2);
    % buts_mid_pos_vals = cell(rows_num, cols_num);
    buts_mid_pos_vals = cell(1, 26);

    offset = 0;
    for i_r = 1:rows_num
        for i_c = 1:cols_num
    % Apply the given homogenous transform to the coordinates of the buttons relative to the top left of the keyboard:
            but_x = x_mid_pos(i_r, i_c);
            but_y = y_mid_pos(i_r);
            but_coords = keyboard_SE3 * [but_x; but_y; 0; 1];
            but_coords = but_coords./but_coords(4, 1);
            buts_mid_pos_vals(i_c + offset) = num2cell(but_coords(1:3, 1)', 2);
            
            if i_c == rows_num_buttons(i_r)
                continue
            end
        end
        offset = rows_num_buttons(i_r);
    end

    % Create a dictionary to map button letters (as keys) to button positions relative to a give coordinate frame:
    buts_mid_pos = containers.Map(rows_button_keys, buts_mid_pos_vals);
end