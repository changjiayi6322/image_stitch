function hat_mat = my_hat(pos)
    hat_mat = [0 -pos(3) pos(2);
               pos(3) 0 -pos(1);
               -pos(2) pos(1) 0];
end

