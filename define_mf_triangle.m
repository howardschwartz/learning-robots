function mf = define_mf_triangle(range, number)
% Define the membnership functions based on the range(x_min, x_max)
% and the number of memebrship function that one wants.
%
x_min = range(1);
x_max = range(2);
%
  if (number == 1)
    mf(1,1) = x_min;
    mf(1,2) = (x_max - x_min)/2;
    mf(1,3) = x_max;
  elseif (number > 1)
     delta = (x_max - x_min)/(number -1);
     %
     mf(1,1) = x_min - delta;
     mf(1,2) = x_min;
     mf(1,3) = x_min + delta;
     %
     for i=2:number
      %
        mf(i,1) = mf(i-1,2);
        mf(i,2) = mf(i-1,3);
        mf(i,3) = mf(i,2) + delta;
       %
     end
  end
end