function set_matlab_utils_path()

path = strrep(mfilename('fullpath'), 'set_matlab_utils_path','');

path = [path '/../../matlab/lib/'];

addpath('utils/');

addpath([path 'math_lib/']);
addpath([path 'io_lib/']);
addpath([path 'dmp_lib/']);

import_math_lib();
import_io_lib();
import_dmp_lib();

end
