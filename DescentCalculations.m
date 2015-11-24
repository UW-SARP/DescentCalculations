%{
Recovery Descent Characteristics
Society for Advanced Rocket Propulsion 2015-2016

Brandt Monson

Following code uses Zach Perry's previous code and plots it in MatLab GUI.
The code is not very sophisticated, but edits will be made to reduce it and
simplify it. A feature to look into is to change the main parachute opening 
time from instant to a gradual increase like what real parachute would be.

%}

function varargout = DescentCalculations(varargin)
% DESCENTCALCULATIONS MATLAB code for DescentCalculations.fig
%      DESCENTCALCULATIONS, by itself, creates a new DESCENTCALCULATIONS or raises the existing
%      singleton*.
%
%      H = DESCENTCALCULATIONS returns the handle to a new DESCENTCALCULATIONS or the handle to
%      the existing singleton*.
%
%      DESCENTCALCULATIONS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DESCENTCALCULATIONS.M with the given input arguments.
%
%      DESCENTCALCULATIONS('Property','Value',...) creates a new DESCENTCALCULATIONS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DescentCalculations_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DescentCalculations_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DescentCalculations

% Last Modified by GUIDE v2.5 15-Nov-2015 16:59:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DescentCalculations_OpeningFcn, ...
                   'gui_OutputFcn',  @DescentCalculations_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before DescentCalculations is made visible.
function DescentCalculations_OpeningFcn(hObject, eventdata, handles, varargin)
% Clears the command window and resets the plotted graphs
clc; arrayfun(@cla,findall(0,'type','axes'));
% Clears the workspace I believe
evalin( 'base', 'clear variables' )

% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DescentCalculations (see VARARGIN)

global coeff_drag_drogue;
global coeff_drag_main;
global area_drogue;
global area_main;
global shock_factor;
global final_altitude;
global initial_altitude;
global init_vel;
global time_step;
global rocket_weight;
global acceleration_gravity;
global timeknot;

% Choose default command line output for DescentCalculations
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes DescentCalculations wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DescentCalculations_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure

varargout{1} = handles.output;



% --- Executes on button press in update_information.
function update_information_Callback(hObject, eventdata, handles)
% hObject    handle to update_information (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global coeff_drag_drogue;
global coeff_drag_main;
global area_drogue;
global area_main;
global shock_factor;
global final_altitude;
global initial_altitude;
global init_vel;
global time_step;
global rocket_mass;
global acceleration_gravity;

coeff_drag_drogue = str2num(get(handles.coeff_drag_drogue,'string'));
coeff_drag_main = str2num(get(handles.coeff_drag_main,'string'));
area_drogue = str2num(get(handles.area_drogue,'string'));
area_main = str2num(get(handles.area_main,'string'));
shock_factor = str2num(get(handles.shock_factor,'string'));
final_altitude = str2num(get(handles.final_altitude,'string'));
initial_altitude = str2num(get(handles.initial_altitude,'string'));
init_vel = str2num(get(handles.init_vel,'string'));
time_step = str2num(get(handles.time_step,'string'));

acceleration_gravity = str2num(get(handles.acceleration_gravity,'string'));
rocket_mass = str2num(get(handles.rocket_weight,'string')) / -(acceleration_gravity);

time = (0:time_step:6)'; %Time
% Standar Atmosphere Table
atm=xlsread('Standard Atmosphere.xlsx', 'a3:f1603');
plot(handles, time, atm);








function plot(handles, time ,atm)
    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Free-fall and drogue parachute deployment
    global coeff_drag_drogue;
    global coeff_drag_main;
    global area_drogue;
    global area_main;
    global shock_factor;
    global final_altitude;
    global initial_altitude;
    global init_vel;
    global time_step;
    global rocket_mass;
    global acceleration_gravity;
    
    accel = acceleration_gravity;
    rocket_height = initial_altitude;
    rocket_vel = init_vel;
    for n=2:length(time)
        accel(n,1) = acceleration_gravity;
        rocket_height(n,1) = init_vel * time_step + 0.5 * accel(n) * time_step^2 + initial_altitude;
        rocket_vel(n,1) = init_vel + accel(n) * time_step;
        init_vel=rocket_vel(n);
        initial_altitude=rocket_height(n);
    end
    rocket_height;
    rocket_vel;
    while abs(accel(n)) >= 0.5
        b = rocket_height(n) / 50;
        c = atm(round(b) + 1,1:end);
        n = n + 1;
        time(n,1) = time(end) + time_step;
        drag(n,1) = coeff_drag_drogue * 0.5 * c(4) * init_vel^2 * area_drogue;
        a1(n,1) = drag(n) / rocket_mass;
        accel(n,1) = acceleration_gravity + a1(n);
        rocket_height(n,1) = init_vel * time_step + 0.5 * accel(n) * time_step^2 + initial_altitude;
        rocket_vel(n,1) = init_vel + accel(n) * time_step;
        init_vel = rocket_vel(n);
        initial_altitude = rocket_height(n);
%         time(n,1) = time(end) + time_step;
    end
    
    position = n;
    
    axes(handles.axes1);
    [AX,H1,H2] = plotyy(time,accel,time,rocket_vel);
    title('accel, Velocity: Apogee-Drogue')
    ylabel(AX(1),'a [ft/s^2]')
    ylabel(AX(2),'v [ft/s]')
    grid on
    xlabel('t [s]')
    
    axes(handles.axes2);
    [AX,H1,H2]=plotyy(time,rocket_height,time,drag);
    title('Altitude, Drag: Apogee-Drogue')
    ylabel(AX(1),'h [ft]')
    ylabel(AX(2),'D [lb_f]')
    xlabel('t [s]')
    grid on

    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %{
    Descent on drogue parachute till main parachute deployment altitude  
    at 2000 AGL
    %}
    dt = 2;
    while rocket_height(n) > (2000 + final_altitude)
        b = rocket_height(n) / 50;
        c = atm(round(b) + 1,1:end);
        n = n + 1;
        time(n,1) = time(end) + dt;
        drag(n,1) = coeff_drag_drogue * 0.5 * c(4) * init_vel^2 * area_drogue;
        a1(n,1) = drag(n) / rocket_mass;
        accel(n,1) =  acceleration_gravity + a1(n);
        rocket_height(n,1) = init_vel * dt + 0.5 * accel(n) * dt^2 + initial_altitude;
        rocket_vel(n,1) = init_vel + accel(n) * dt;
        init_vel = rocket_vel(n);
        initial_altitude = rocket_height(n);
        time(n,1) = time(end) + dt;
    end
    
    axes(handles.axes3)
    [AX,H1,H2] = plotyy(time,accel,time,rocket_vel);
    title('accel, Velocity: Apogee-2000ft')
    ylabel(AX(1),'a [ft/s^2]')
    ylabel(AX(2),'v [ft/s]')
    % xlabel('t [s]')
    grid on
    
    axes(handles.axes4)
    [AX,H1,H2] = plotyy(time,rocket_height,time,drag);
    title('Altitude, Drag: Apogee-2000ft')
    ylabel(AX(1),'h [ft]')
    ylabel(AX(2),'D [lb_f]')
    xlabel('t [s]')
    grid on

    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    %{ 
    Main parachute deployment with opening shock factor approaching 
    one (shock_factor->1) after initial shock calculation to determine accurate descent
    velocity at landing
    %}
    initial_altitude = rocket_height(n);
    rocket_vel(n) = rocket_vel(n);
    a2 = 1;
    dt = 0.0025;
    n1 = n;
    while abs(a2)>= 0.5
        b = rocket_height(n) / 50;
        c = atm(round(b) + 1,1:end);
        n = n + 1;
        time(n,1) = time(end) + dt;
        drag(n,1) = coeff_drag_main * 0.5 * c(4) * init_vel^2 * area_main * shock_factor;
        a1(n,1) = drag(n) / rocket_mass;
        accel(n,1) = acceleration_gravity + a1(n);
        rocket_height(n,1) = init_vel * dt + 0.5 * accel(n) * dt^2 + initial_altitude;
        rocket_vel(n,1) = init_vel + accel(n) * dt;
        init_vel = rocket_vel(n);
        initial_altitude = rocket_height(n);
        a2 = accel(n);
        if shock_factor < 1
            shock_factor = shock_factor + 0.00025;
        else
        end
    end

    axes(handles.axes5)
    [AX,H1,H2] = plotyy(time(n1:end),accel(n1:end),time(n1:end),rocket_vel(n1:end));
    title('accel, Velocity: Main Deployment')
    ylabel(AX(1),'a [ft/s^2]')
    ylabel(AX(2),'v [ft/s]')
    % xlabel('t [s]')
    grid on
    
    axes(handles.axes6)
    [AX,H1,H2] = plotyy(time(n1:end),rocket_height(n1:end),time(n1:end),drag(n1:end));
    title('Altitude, Drag: Main')
    ylabel(AX(1),'h [ft]')
    ylabel(AX(2),'D [lb_f]')
    xlabel('t [s]')
    grid on

    %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    % Final descent to ground level

    dt=0.05;
    while rocket_height(n) > (final_altitude)
        b = rocket_height(n)/50;
        c = atm(round(b) + 1,1:end);
        n = n + 1;
        time(n,1) = time(end) + dt;
        drag(n,1) = coeff_drag_main * 0.5 * c(4) * init_vel^2 * area_main;
        a1(n,1) = drag(n) / rocket_mass;
        accel(n,1) = acceleration_gravity + a1(n);
        rocket_height(n,1) = init_vel * dt + 0.5 * accel(n) * dt^2 + initial_altitude;
        rocket_vel(n,1) = init_vel + accel(n) * dt;
        init_vel = rocket_vel(n);
        initial_altitude = rocket_height(n);
        a2 = accel(n);
    end

    axes(handles.axes7)
    [AX,H1,H2] = plotyy(time,accel,time,rocket_vel);
    title('accel, Velocity: Apogee-Ground')
    ylabel(AX(1),'a [ft/s^2]')
    ylabel(AX(2),'v [ft/s]')
    xlabel('t [s]')
    grid on
    
    axes(handles.axes8)
    [AX,H1,H2] = plotyy(time,rocket_height,time,drag);
    title('Altitude, Drag: Apogee-Ground')
    ylabel(AX(1),'h [ft]')
    ylabel(AX(2),'D [lb_f]')
    xlabel('t [s]')
    grid on

































%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% THE FOLLOWING IS FOR INPUT BOXES TO BE SETUP NO NEED TO EDIT %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function coeff_drag_drogue_Callback(hObject, eventdata, handles)
% hObject    handle to coeff_drag_drogue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of coeff_drag_drogue as text
%        str2double(get(hObject,'String')) returns contents of coeff_drag_drogue as a double


% --- Executes during object creation, after setting all properties.
function coeff_drag_drogue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to coeff_drag_drogue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function coeff_drag_main_Callback(hObject, eventdata, handles)
% hObject    handle to coeff_drag_main (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of coeff_drag_main as text
%        str2double(get(hObject,'String')) returns contents of coeff_drag_main as a double


% --- Executes during object creation, after setting all properties.
function coeff_drag_main_CreateFcn(hObject, eventdata, handles)
% hObject    handle to coeff_drag_main (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function area_drogue_Callback(hObject, eventdata, handles)
% hObject    handle to area_drogue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of area_drogue as text
%        str2double(get(hObject,'String')) returns contents of area_drogue as a double


% --- Executes during object creation, after setting all properties.
function area_drogue_CreateFcn(hObject, eventdata, handles)
% hObject    handle to area_drogue (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function area_main_Callback(hObject, eventdata, handles)
% hObject    handle to area_main (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of area_main as text
%        str2double(get(hObject,'String')) returns contents of area_main as a double


% --- Executes during object creation, after setting all properties.
function area_main_CreateFcn(hObject, eventdata, handles)
% hObject    handle to area_main (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function shock_factor_Callback(hObject, eventdata, handles)
% hObject    handle to shock_factor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of shock_factor as text
%        str2double(get(hObject,'String')) returns contents of shock_factor as a double


% --- Executes during object creation, after setting all properties.
function shock_factor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to shock_factor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function final_altitude_Callback(hObject, eventdata, handles)
% hObject    handle to final_altitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of final_altitude as text
%        str2double(get(hObject,'String')) returns contents of final_altitude as a double


% --- Executes during object creation, after setting all properties.
function final_altitude_CreateFcn(hObject, eventdata, handles)
% hObject    handle to final_altitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function initial_altitude_Callback(hObject, eventdata, handles)
% hObject    handle to initial_altitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of initial_altitude as text
%        str2double(get(hObject,'String')) returns contents of initial_altitude as a double


% --- Executes during object creation, after setting all properties.
function initial_altitude_CreateFcn(hObject, eventdata, handles)
% hObject    handle to initial_altitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function init_vel_Callback(hObject, eventdata, handles)
% hObject    handle to init_vel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of init_vel as text
%        str2double(get(hObject,'String')) returns contents of init_vel as a double


% --- Executes during object creation, after setting all properties.
function init_vel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to init_vel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function time_step_Callback(hObject, eventdata, handles)
% hObject    handle to time_step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of time_step as text
%        str2double(get(hObject,'String')) returns contents of time_step as a double


% --- Executes during object creation, after setting all properties.
function time_step_CreateFcn(hObject, eventdata, handles)
% hObject    handle to time_step (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function rocket_weight_Callback(hObject, eventdata, handles)
% hObject    handle to rocket_weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rocket_weight as text
%        str2double(get(hObject,'String')) returns contents of rocket_weight as a double


% --- Executes during object creation, after setting all properties.
function rocket_weight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rocket_weight (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function acceleration_gravity_Callback(hObject, eventdata, handles)
% hObject    handle to acceleration_gravity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of acceleration_gravity as text
%        str2double(get(hObject,'String')) returns contents of acceleration_gravity as a double


% --- Executes during object creation, after setting all properties.
function acceleration_gravity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to acceleration_gravity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
