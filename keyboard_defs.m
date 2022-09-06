
rows_num_buttons = [10; 9; 7];
rows_button_keys = {'Q' 'W' 'E' 'R' 'T' 'Y' 'U' 'I' 'O' 'P',...
                    'A' 'S' 'D' 'F' 'G' 'H' 'J' 'K' 'L',...
                    'Z' 'X' 'C' 'V' 'B' 'N' 'M'};
button_size = [30 30];
button_gap = 3;
% Each row contains (x,y) for each row on the keyboard:
rows_start_px = [55 65; 63 97; 80 130];
buttons_x_offsets = buttons_x_offsets_calc(rows_start_px(:, 1), rows_num_buttons, button_size(1), button_gap);
buttons_x_middle_pxs = arrayfun(@(start_px) start_px + button_size(1)/2, buttons_x_offsets);
buttons_y_middle_pxs = -1 * (rows_start_px(:, 2) + button_size(2)/2);

keyboard_img_size = [487 201];
% Size in mm:
keyboard_phy_size = [120 290];
keyboard_scale = keyboard_phy_size./keyboard_img_size;

buttons_x_mid_pos = arrayfun(@(mid_px) mid_px*keyboard_scale(1), buttons_x_middle_pxs);
buttons_y_mid_pos = arrayfun(@(mid_px) mid_px*keyboard_scale(2), buttons_y_middle_pxs);

imshow('keyboard.jpg');
hold on
plot(buttons_x_middle_pxs, buttons_y_middle_pxs)
hold off

function x_offsets = buttons_x_offsets_calc(start_pxs, num_keys,...
                                            button_width, button_gap)
    x_offsets = zeros(size(start_pxs, 1), max(num_keys));
    for i_r = 1:size(start_pxs, 1)
        offset = start_pxs(i_r, 1);
        x_offsets(i_r, 1) = offset;
        for i_c = 2:num_keys(i_r)
            x_offsets(i_r, i_c) = offset + button_width + button_gap;
            offset = x_offsets(i_r, i_c);
        end
    end
end