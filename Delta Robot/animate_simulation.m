function animate_simulation(n_total,X,q);
    La = 1;
    Lb = 1.25*sqrt(2);
    Ra= 0.5;
    Rb = 0.25;
    R = Ra-Rb;
    phi = [0; 120*pi/180; 240*pi/180];
    
    R_t = zeros(3,3,3);
    r_t = zeros(3,3,3);
    B = zeros(3,3,n_total);
    C = zeros(3,3,n_total);
    A = zeros(3,3);
    a = zeros(3,3);

    for i = 1:3
        R_t(:,:,i) = [cos(phi(i)) -sin(phi(i)) 0; sin(phi(i)) cos(phi(i)) 0; 0 0 1];
        r_t (:,:,i) = 2*[cos(phi(i) + deg2rad(60)) -sin(phi(i) + deg2rad(60)) 0; sin(phi(i) + deg2rad(60)) cos(phi(i) + deg2rad(60)) 0; 0 0 1];    
        A(:,i) = R_t(:,:,i)*[Ra;0;0];
        a(:,i) = r_t(:,:,i)*[Ra;0;0];
    end
    
    figure('Name','Simulation')
    for n=1:n_total-1

        for i = 1:3
            B(:,i,n) = R_t(:,:,i)*[La*cos(q(i,n));0;-La*sin(q(i,n))];
            C(:,i,n) = [X(1,n) + R*cos(phi(i)); X(2,n) + R*sin(phi(i)); X(3,n)];
        end

        hold on;

        plot3([a(1,1) a(1,2) a(1,3) a(1,1)],[a(2,1) a(2,2) a(2,3) a(2,1)],[a(3,1) a(3,2) a(3,3) a(3,1)])
        plot3([X(1,n) C(1,1,n) C(1,2,n) C(1,3,n) C(1,1,n)],[X(2,n) C(2,1,n) C(2,2,n) C(2,3,n) C(2,1,n)],[X(3,n) C(3,1,n) C(3,2,n) C(3,3,n) C(3,1,n)])

        plot3([A(1,1) B(1,1,n) C(1,1,n)],[A(2,1) B(2,1,n) C(2,1,n)],[A(3,1) B(3,1,n) C(3,1,n)])
        plot3([A(1,2) B(1,2,n) C(1,2,n)],[A(2,2) B(2,2,n) C(2,2,n)],[A(3,2) B(3,2,n) C(3,2,n)])
        plot3([A(1,3) B(1,3,n) C(1,3,n)],[A(2,3) B(2,3,n) C(2,3,n)],[A(3,3) B(3,3,n) C(3,3,n)])

        axis square
        xlabel('X postition')
        ylabel('Y postition')
        zlabel('Z postition')
        xlim([-1.5 1.5])
        ylim([-1.5 1.5])
        zlim([-3 0])
        view(3)
        grid on;
        drawnow();
        pause(0.5)

        if mod(n,3)
         clf
        end
        
    end
    hold off
end