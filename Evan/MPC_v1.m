clear all
close all
clc
%Inputs
L1=4; %Truck length
L2=1;  %Car Length

Dynamic=1;  % 1=for action, 0=just to see set up

rad=pi/180;  %radian to degree conversion

v0=0;   %intiial velocity
Phi0=0   *rad;   %initial Steering angle
Omega0=45  *rad;   %initial angle of truck relative to x-axis, 0 deg is North, 90 deg is east
Theta0=0  *rad; %iniitial angle of trailer to truck, 0 is inline

N=1000; %number of steps
Dt=1; %time step = 1 second to start


%%%Code


y0(1) =0;  %Truck Front Axle - y
y0(2) = -cos(-Omega0)*L1;%Truck Back Axle - y
y0(3) = y0(2)-cos(Theta0+Omega0)*L2;%Trailer Back Axle - y

x0(1)=0;%Truck Front Axle - x
x0(2)=-sin(Omega0)*L1;%Truck Back Axle - x
x0(3) = x0(2)-sin(Theta0+Omega0)*L2;%Trailer Back Axle - x



a=zeros(1,N); %acceleration
B=zeros(1,N); %steering angle change
y=zeros(3,N);%y co-ordinate of 3 axles
x=zeros(3,N);%x co-ordinate of 3 axles
v=zeros(1,N); % velocity
Phi=zeros(1,N); %Steering angle
Omega=zeros(1,N); %angle of truck relative to x-axis, 0 deg is North, 90 deg is east
Theta=zeros(1,N); %angle of trailer to truck, 0 is inline



%SET up inputs
Nframes=600;
time=0.0; %time between frames for animation



%just set up some random inputs for demonstration
for i=1:400
    v(i)=0.1;  %goes forward
end
for i=400:600
    v(i)=-0.1;  %goes backwards
end
for i=1:150
    Phi(i)=10*rad;%turns slight right
end
for i=150:300
    Phi(i)=45*rad; %turns stronger right
end
for i=300:450
    Phi(i)=10*rad;%turns slight right
end
for i=450:600
    Phi(i)=-45*rad;%turns strong left
end



%%%%Step 1
for i=1:3 % assign initial values 
y(i,1)=y0(i);
x(i,1)=x0(i);
end
v(1)=v0;
Phi(1)=Phi0;
Omega(1)=Omega0;
Theta(1)=Theta0;



%code breaks down when truck goes perfectly vertical or perfectly
%horizontal hence the cases at the bottom.
limit=10^6; %substitute for infinity.



%%Step k+1
for i=1:N-1
    xd1(i)=x(1,i)-x(2,i); %Truckk Front Axls differnce in x-co-ord
    yd1(i)=y(1,i)-y(2,i);%Truckk Front Axls differnce in y-co-ord
    Omega(i)=-atan2(yd1(i),xd1(i))+pi/2; %Truckk Global angle next step


    x(1,i+1)=x(1,i)+Dt*v(i)*sin(Phi(i)+Omega(i)); %Truckk Front Axle x next step
    y(1,i+1)=y(1,i)+Dt*v(i)*cos(Phi(i)+Omega(i));%Truckk Front Axle y next step


    %Truck read axle calculation
    m_truck(i)=(y(2,i)-y(1,i))/(x(2,i)-x(1,i));%Truckk slope (global)

    if m_truck(i) < limit && m_truck(i) > -limit && m_truck(i) ~=0 %if slope is not vertical or horizontal
        %fprintf("no")
        c_truck(i)=y(1,i)-m_truck(i)*x(1,i); %y=mx+c 
    
        %Ax+By+C=0
        A1(i)=-m_truck(i); 
        B1(i)=1;
        C1(i)=m_truck(i)*x(1,i)-y(1,i);
    

        %ay^2 + by +c = 0
        a_q1(i)=(1/(A1(i)^2))+1;
        b_q1(i)=((2*(C1(i)+A1(i)*x(1,i+1)))/(A1(i)^2))-2*y(1,i+1);
        c_q1(i)=((C1(i)+A1(i)*x(1,i+1))^2)/(A1(i)^2)+y(1,i+1)^2-L1^2;
    

        %y=(-b+/-sqrt(b^2-4ac))/2a
        y_2p(i+1)=(-b_q1(i)+sqrt(b_q1(i)^2-4*a_q1(i)*c_q1(i)))/(2*a_q1(i));%y plus answer
        y_2m(i+1)=(-b_q1(i)-sqrt(b_q1(i)^2-4*a_q1(i)*c_q1(i)))/(2*a_q1(i));%y minus answer
    
        x_2p(i+1)=(-y_2p(i+1)-C1(i))/A1(i);%x plus answer
        x_2m(i+1)=(-y_2m(i+1)-C1(i))/A1(i);%x minus answer
    
        %distance between truck rear axle from step k to k+1 for both plus
        %and minus options
        d1_p(i) = sqrt((y(2,i)-y_2p(i+1))^2+(x(2,i)-x_2p(i+1))^2);
        d1_m(i) = sqrt((y(2,i)-y_2m(i+1))^2+(x(2,i)-x_2m(i+1))^2);
    

        %chosee the plus or minus option which is less
        if d1_p(i)<d1_m(i) 
            y(2,i+1)=y_2p(i+1);
            x(2,i+1)=x_2p(i+1);
            pm1(i)=1;
        else
            y(2,i+1)=y_2m(i+1);
            x(2,i+1)=x_2m(i+1);
            pm1(i)=-1;
        end


        %if the slop was vertical, i.e. negative or positive infiinity
        %the calculation is slightly different between same idea.
    elseif m_truck(i)<-limit || m_truck(i)>limit 
        vert_d1_p = abs(y(2,i)-(y(1,i+1)+sqrt(L1^2-(x(1,i+1)-x(2,i))^2)));
        vert_d1_m = abs(y(2,i)-(y(1,i+1)-sqrt(L1^2-(x(1,i+1)-x(2,i))^2)));
        
        if vert_d1_p<vert_d1_m
        x(2,i+1)=x(2,i);
        y(2,i+1)=(y(1,i+1)+sqrt(L1^2-(x(1,i+1)-x(2,i))^2));
        else
        x(2,i+1)=x(2,i);
        y(2,i+1)=(y(1,i+1)-sqrt(L1^2-(x(1,i+1)-x(2,i))^2));
        end
    else 
        %if the slop was horizontal, slope = 0, 
        horz_d1_p = abs(x(2,i)-(x(1,i+1)+sqrt(L1^2-(y(1,i+1)-y(2,i))^2)));
        horz_d1_m = abs(x(2,i)-(x(1,i+1)-sqrt(L1^2-(y(1,i+1)-y(2,i))^2)));
        
        if horz_d1_p>horz_d1_m
        y(2,i+1)=y(2,i);
        x(2,i+1)=(x(1,i+1)+sqrt(L1^2-(y(1,i+1)-y(2,i))^2));
        else
        y(2,i+1)=y(2,i);
        x(2,i+1)=(x(1,i+1)-sqrt(L1^2-(y(1,i+1)-y(2,i))^2));
        end
    end



    %%%%%%%%%%%trailer rear axle calculation


    m_trailer(i)=(y(3,i)-y(2,i))/(x(3,i)-x(2,i));
    if m_trailer(i) < limit && m_trailer(i) > -limit && m_trailer(i) ~=0
        %fprintf("no")
        c_trailer(i)=y(2,i)-m_trailer(i)*x(2,i);
    
        A2(i)=-m_trailer(i);
        B2(i)=1;
        C2(i)=m_trailer(i)*x(2,i)-y(2,i);
    
        a_q2(i)=(1/(A2(i)^2))+1;
        b_q2(i)=((2*(C2(i)+A2(i)*x(2,i+1)))/(A2(i)^2))-2*y(2,i+1);
        c_q2(i)=((C2(i)+A2(i)*x(2,i+1))^2)/(A2(i)^2)+y(2,i+1)^2-L2^2;
    
        y_3p(i+1)=(-b_q2(i)+sqrt(b_q2(i)^2-4*a_q2(i)*c_q2(i)))/(2*a_q2(i));
        y_3m(i+1)=(-b_q2(i)-sqrt(b_q2(i)^2-4*a_q2(i)*c_q2(i)))/(2*a_q2(i));
    
        x_3p(i+1)=(-y_3p(i+1)-C2(i))/A2(i);
        x_3m(i+1)=(-y_3m(i+1)-C2(i))/A2(i);
    
    
        d2_p(i) = sqrt((y(3,i)-y_3p(i+1))^2+(x(3,i)-x_3p(i+1))^2);
        d2_m(i) = sqrt((y(3,i)-y_3m(i+1))^2+(x(3,i)-x_3m(i+1))^2);
    
        if d2_p(i)<d2_m(i)
            y(3,i+1)=y_3p(i+1);
            x(3,i+1)=x_3p(i+1);
            pm2(i)=1;
        else
            y(3,i+1)=y_3m(i+1);
            x(3,i+1)=x_3m(i+1);
            pm2(i)=-1;
        end
    elseif m_trailer(i)<-limit || m_trailer(i)>limit 
        vert_d2_p = abs(y(3,i)-(y(2,i+1)+sqrt(L2^2-(x(2,i+1)-x(3,i))^2)));
        vert_d2_m = abs(y(3,i)-(y(2,i+1)-sqrt(L2^2-(x(2,i+1)-x(3,i))^2)));
        
        if vert_d2_p<vert_d2_m
        x(3,i+1)=x(3,i);
        y(3,i+1)=(y(2,i+1)+sqrt(L2^2-(x(2,i+1)-x(3,i))^2));
        else
        x(3,i+1)=x(3,i);
        y(3,i+1)=(y(2,i+1)-sqrt(L2^2-(x(2,i+1)-x(3,i))^2));
        end
    else 

        horz_d2_p = abs(x(3,i)-(x(2,i+1)+sqrt(L2^2-(y(2,i+1)-y(3,i))^2)));
        horz_d2_m = abs(x(3,i)-(x(2,i+1)-sqrt(L2^2-(y(2,i+1)-y(3,i))^2)));
        
        if horz_d2_p>horz_d2_m
        y(3,i+1)=y(3,i);
        x(3,i+1)=(x(2,i+1)+sqrt(L2^2-(y(2,i+1)-y(3,i))^2));
        else
        y(3,i+1)=y(3,i);
        x(3,i+1)=(x(2,i+1)-sqrt(L2^2-(y(2,i+1)-y(3,i))^2));
        end
    end
end





%%Graphing
t=3; % lien thickness
if Dynamic == 0
    Nframes=1;
end

 %setting plot margins
allX = [x(1,:)', x(2,:)', x(3,:)'];
allY = [y(1,:)', y(2,:)', y(3,:)'];
x_U_lim = max(max(allX));
x_L_lim = min(min(allX));
y_U_lim = max(max(allY));
y_L_lim = min(min(allY));


for i = 1:Nframes
    cla; % clear frame
    hold on
    grid on

    xlim([x_L_lim, x_U_lim]);
    ylim([y_L_lim, y_U_lim]);
    axis equal
    % Points
    P1 = [x(1,i),y(1,i)];
    P2 = [x(2,i),y(2,i)];
    P3 = [x(3,i),y(3,i)];

     % --- Draw segments ---

    plot([P1(1), P2(1)], [P1(2), P2(2)], 'b-', 'LineWidth', t); %Truck
    plot([P2(1), P3(1)], [P2(2), P3(2)], 'r-', 'LineWidth', t); %Trailer


        % --- Draw joints ---
    plot(P1(1), P1(2), 'ko', 'MarkerFaceColor','k','MarkerSize',6);
    plot(P2(1), P2(2), 'ko', 'MarkerFaceColor','k','MarkerSize',6);
    plot(P3(1), P3(2), 'ko', 'MarkerFaceColor','k','MarkerSize',6);


    drawnow limitrate nocallbacks
    pause(time)
 
    hold off
end
