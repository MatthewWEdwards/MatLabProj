function [result, status] = PYTHON(varargin)
%PYTHON Execute PYTHON command and return the result.
%   PYTHON(PYTHONFILE) calls PYTHON script specified by the file PYTHONFILE
%   using appropriate PYTHON executable.
%
%   PYTHON(PYTHONFILE,ARG1,ARG2,...) passes the arguments ARG1,ARG2,...
%   to the PYTHON script file PYTHONFILE, and calls it by using appropriate
%   PYTHON executable.
%
%   RESULT=PYTHON(...) outputs the result of attempted PYTHON call.  If the
%   exit status of PYTHON is not zero, an error will be returned.
%
%   [RESULT,STATUS] = PYTHON(...) outputs the result of the PYTHON call, and
%   also saves its exit status into variable STATUS.
%
%   If the PYTHON executable is not available, it can be downloaded from:
%     http://www.cpan.org
%
%   See also SYSTEM, JAVA, MEX.

%   Copyright 1990-2012 The MathWorks, Inc.
    
cmdString = '';

% Add input to arguments to operating system command to be executed.
% (If an argument refers to a file on the MATLAB path, use full file path.)
for i = 1:nargin
    thisArg = varargin{i};
    if ~ischar(thisArg)
        error(message('MATLAB:PYTHON:InputsMustBeStrings'));
    end
    if i==1
        if exist(thisArg, 'file')==2
            % This is a valid file on the MATLAB path
            if isempty(dir(thisArg))
                % Not complete file specification
                % - file is not in current directory
                % - OR filename specified without extension
                % ==> get full file path
                thisArg = which(thisArg);
            end
        else
            % First input argument is PYTHONFile - it must be a valid file
            error(message('MATLAB:PYTHON:FileNotFound', thisArg));
        end
    end

    % Wrap thisArg in double quotes if it contains spaces
    if isempty(thisArg) || any(thisArg == ' ')
        thisArg = ['"', thisArg, '"']; %#ok<AGROW>
    end

    % Add argument to command string
    cmdString = [cmdString, ' ', thisArg]; %#ok<AGROW>
end

% Execute PYTHON script
if isempty(cmdString)
    error(message('MATLAB:PYTHON:NoPYTHONCommand'));
elseif ispc
    % PC
    if(isdeployed)
        % Check to see if PYTHON is on the path. 
        % If it isn't, this will return a non 0 status
        [status, ~] = dos('PYTHON -v');
        if(status ~=0)
            error(message('MATLAB:PYTHON:NoExecutable'));
        end 
    end
    PYTHONCmd = fullfile(matlabroot, 'C:\Documents\Anaconda\envs\p2');
    cmdString = ['PYTHON' cmdString];
    PYTHONCmd = ['set PATH=',PYTHONCmd, ';%PATH%&' cmdString];
    [status, result] = dos(PYTHONCmd);
else
    % UNIX
    [status, ~] = unix('which PYTHON');
    if (status == 0)
        cmdString = ['PYTHON', cmdString];
        [status, result] = unix(cmdString);
    else
        error(message('MATLAB:PYTHON:NoExecutable'));
    end
end



