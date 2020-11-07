close all;
clear all;
clc

%Denavit - Hartenberg
DH= [0.000 0.780  0.410 pi/2  0;
     0.000 0.000  1.075 0.000 0;
     0.000 0.000  0.165 pi/2  0;
     0.000 1.056  0.000 pi/2  0;
     0.000 0.000  0.000 -pi/2 0;
     0.000 0.250  0.000 0.000 0];

%Creamos el objeto IRB7600 
IRB7600= SerialLink(DH,'name','ABB IRB7600');

%Posicion inicial: De reposo
q0=[0.3309    2.4763   -1.1892    2.1270    0.5639   -2.1385];
% q0=[(45*pi)/180 (125*pi)/180 (-65*pi)/180 0 (-30*pi)/180  0];

%IRB7600.plot3d(q0,'path',pwd);

%Limites articulares (El fabricante los especifica en la Pag.57 del Datasheet)
%Estos fueron asignados en funcion de nuestra aplicacion.
IRB7600.qlim=[(-180*pi)/180 (180*pi)/180
              (-15*pi)/180 (135*pi)/180;
              (-65*pi)/180 (220*pi)/180 ;              
              (0*pi)/180 (360*pi)/180;
              (-130*pi)/180 (130*pi)/180 ;
              (0*pi)/180 (360*pi)/180];
          
%Con esta funcion controlamos la traslacion y rotacion de la base      
IRB7600.base = trotx(0);

%Se asume que la herramienta tiene un largo de 1.30 [mts]
%Con esta funcion modificamos la traslacion del efector final.
IRB7600.tool = transl(0,0,0)*trotx(0);

disp('Ingrese que operacion desea realizar:');
disp('1.Plotear espacio de trabajo');
disp('2.Calcular Cinematica directa');
disp('3.Calcular Cinematica inversa');
disp('4.Calcular Matriz Jacobiana Directa');
disp('5.Planificacion de Trayectorias');


%==========================================================================
operaciones=input('');

switch operaciones
    case 1      %Ploteo espacio de trabajo PARA POSICION DE REPOSO -> q1 
        q1=[(90*pi)/180 (0*pi)/180 (90*pi)/180 (0*pi)/180 (90*pi)/180  (90*pi)/180];
        for i=((-125*pi)/180):pi/50:((125*pi)/180)
        q1(1)=i;
        T=IRB7600.fkine(q1);
        scatter3(T(1,4),T(2,4),T(3,4), 'r.');
        hold on
        axis([-5 5 -5 5 -5 5]);
        end
        IRB7600.plot(q1);
        %axis([-5 5 -5 5 -5 5]);
        IRB7600.teach
        
%==========================================================================       

    case 2     %Cinematica directa
        
        % Tin: Tranformacion de ref (la anterior)
        % Tout: Tranfomacion en la union actual     
        Tin=IRB7600.base;   %Base de referencia. 
        Tout=0;
        %Metodo de las Transformaciones homogeneas
        for i=1:6 
        Tout=Tin*trotz(q0(i))*transl(0,0,DH(i,2))*transl(DH(i,3),0,0)*trotx(DH(i,4));
        disp('Tranformacion:');
        disp(Tout);
        %Actualizacion de la variable
        Tin=Tout;
        end
        
        %fkine -> Forward Kinematic (Cinematica directa)
        %Retorna cada una de las transformaciones de cada union respecto a la
        %anterior, para una determinada configurarcion de variables articulares "q"
     
        [Ttotal,all]=IRB7600.fkine(q0);
        
        %El metodo A(i,q(i)) retorna la tranformacion de las uniones que se le
        %indique  Por ej: A([1 2 3], q)->> T: de la articulacion 3 respecto a 2
        
        S=IRB7600.A([1 2 3 4 5 6],q0)*transl(0,0,1.30);
        
        IRB7600.plot(q0,'workspace',[-3 3 -3 3 -5 5]);
        IRB7600.teach

%==========================================================================        
    case 3   %Cinematica Inversa

        %Este vector de coordenadas se obtuvo ubicando el robot con el teach en
        %una posicion aproximada en la cual se encontraria cuando este operando
        %con el horno

        q_inv=[(45*pi)/180 (10*pi)/180 (80*pi)/180 0 (0*pi)/180  (0*pi)/180];    

        %Por C-D se obtiene una matriz de tranformacion 4x4 que contiene
        %posicion y orientacion del extremo.. Nos sirve como dato de entrada
        %paara el calculo de la cinematica inversa


       % T=IRB7600.A([1 2 3 4 5 6],q_inv)*transl(0,0,1.30);
        %T=[1 0 0 2; 0 -1 0 0.3; 0 0 -1 1.55; 0 0 0  1];
        %----------
        T=IRB7600.fkine(q_inv);
        
        Px=T(1,4);
        Py=T(2,4);
        Pz=T(3,4);

        %Valores de longitud
        l2=DH(2,3);
        l3=DH(4,2);

        W = T(1:3,3);  %COMPONENTES DE ROTAZION DE Z  (orientacion)
        d6=0.250; %+1.300;  %consideramos 0.250 + 1.300 de la herramienta
        % Pm: posicion muneca
        
        %Metodo de Pieper (para q1)
        Pm =[Px Py Pz]' - d6*W ;
        Pm(4)=1; %escalado

        d0w=sqrt(Pm(1)^2+Pm(2)^2+Pm(3)^2);  %distancia desde la base a la muneca
        q1(1)=atan2(Pm(2),Pm(1)); 
        q1(2)=q1(1)+pi;
        
        %Calculo de q2              
        
        T011=IRB7600.A([1],q1(1)); %Matriz de transformacion de 0 a 1 
        T012=IRB7600.A([1],q1(2)); 
  
        Pm011=inv(T011)*Pm;
        Pm012=inv(T012)*Pm;

        d16=sqrt(Pm011(1)^2+Pm011(2)^2);     
        d16pi=sqrt(Pm012(1)^2+Pm012(2)^2);

         %Por geometria
         
         ld=sqrt(1.056^2+0.165^2);
         l2=1.075;
         d4=1.056;
         a3=0.165;
         
         beta1=atan2(Pm011(2),Pm011(1));  %beta es siempre fijo
         beta2=atan2(Pm012(2),Pm012(1));
         
         %Teorema coseno (Para q1(1))
         
         gamma1=real(acos((l2^2+d16^2-ld^2)/(2*l2*d16)));
         
         q2(1)=beta1+gamma1;  %Codo arriba
         q2(2)=beta1-gamma1;  %Codo abajo
         
         %Teoroma coseno (Para q1(2))
         
         gamma2=real(acos((l2^2+d16^2-ld^2)/(2*l2*d16pi)));
         
         q2(3)=beta2+gamma2;  %Codo arriba
         q2(4)=beta2-gamma2;  %Codo abajo 

         %Calculo de q3
         
         %Fijamos un angulo phi
         
         phi=atan2(d4,a3); %offset por geometria
        
         eta1=real(acos((l2^2+ld^2-d16^2)/(2*l2*ld)));
         
         q3(1)=pi-phi-eta1;
         q3(2)=pi-phi+eta1;
         
         eta2=real(acos((l2^2+ld^2-d16pi^2)/(2*l2*ld)));
         
        q3(3)=pi-phi-eta2;
        q3(4)=pi-phi+eta2;         
         
        % Muneca, ultimas 3 articulaciones
        T031=IRB7600.A([1,2,3],[q1(1) q2(1) q3(1)]);
        T032=IRB7600.A([1,2,3],[q1(1) q2(2) q3(2)]);   
         
        vx3=T031(1:3,1);
        vy3=T031(1:3,2);
        vz3=T031(1:3,3);
        
        vx31=T032(1:3,1);
        vy31=T032(1:3,2);
        vz31=T032(1:3,3);
        
        %El vector  z6=T(1:3,3) coincide con z5
        vx6=T(1:3,1);
        vz5=T(1:3,3); 
        
        vz4=cross(vz3, vz5);	
        vz41=cross(vz31, vz5);	
        disp('Muneca arriba =1 ; Muneca abajo = 0');
        wrist=input('');
  
        %Cuando vz3 y vz5 son paralelos el producto vectorial vale 0 
  
        if (norm(vz4)|norm(vz41)) <= 0.000001
            if wrist == 1 %Muneca arriba
                q4(1)=0;
            elseif wrist == 0
                q4(1)=pi; %Muneca abajo
            else
                disp("Error");
                return;
            end 
        else
            
        %Esta es la solucion mas frequente(que no se alineen los ejes)
        cosq4=wrist*dot(-vy3,vz4);
        sinq4=wrist*dot(vx3,vz4);
        q4(1)=atan2(sinq4, cosq4);

        cosq4=wrist*dot(vy3,vz4);
        sinq4=wrist*dot(-vx3,vz4);
        q4(2)=atan2(sinq4, cosq4);
        %-----
        cosq4=wrist*dot(-vy31,vz41);
        sinq4=wrist*dot(vx31,vz41);
        q4(3)=atan2(sinq4, cosq4);

        cosq4=wrist*dot(vy31,vz41);
        sinq4=wrist*dot(-vx31,vz41);
        q4(4)=atan2(sinq4, cosq4);
            
        end

        %Propagamos q4 para calcular q5 
         
        T041=IRB7600.A([1,2,3,4],[q1(1) q2(1) q3(1) q4(1)]);
        T042=IRB7600.A([1,2,3,4],[q1(1) q2(2) q3(2) q4(2)]);
          
        vx4=T041(1:3,1);
        vy4=T041(1:3,2);
        
        vx41=T042(1:3,1);
        vy41=T042(1:3,2);
        
        %q5 
        cosq5=dot(vy4,vz5);
        sinq5=dot(-vx4,vz5);
        q5(1)=atan2(sinq5, cosq5);
       
        cosq5=dot(-vy4,vz5);
        sinq5=dot(+vx4,vz5);
        q5(2)=atan2(sinq5, cosq5);
        

        cosq5=dot(vy41,vz5);
        sinq5=dot(-vx41,vz5);
        q5(3)=atan2(sinq5, cosq5);
       
        cosq5=dot(-vy41,vz5);
        sinq5=dot(+vx41,vz5);
        q5(4)=atan2(sinq5, cosq5);
        
        %Propagamos q5 para calcular q6 
        T051=IRB7600.A([1,2,3,4,5],[q1(1) q2(1) q3(1) q4(1) q5(1)]);
        T052=IRB7600.A([1,2,3,4,5],[q1(1) q2(2) q3(2) q4(2) q5(2)]);
          
        vx5=T051(1:3,1);
        vy5=T051(1:3,2);
        
                  
        vx51=T052(1:3,1);
        vy51=T052(1:3,2);
        %q6
        cosq6=dot(vx6,vx5);
        sinq6=dot(vx6,vy5);
        q6(1)=atan2(sinq6, cosq6);
        
        cosq6=dot(vx6,-vx5);
        sinq6=dot(vx6,-vy5);
        q6(2)=atan2(sinq6, cosq6);  
        
        cosq6=dot(vx6,vx51);
        sinq6=dot(vx6,vy51);
        q6(3)=atan2(sinq6, cosq6);  
        
        cosq6=dot(vx6,-vx51);
        sinq6=dot(vx6,-vy51);
        q6(4)=atan2(sinq6, cosq6); 
        
%==========================================================================
    case 4 %Jacobiano
       disp('1.Simbolico');
       disp('2.Numerico');
       opcion=input('');
       switch opcion
           case 1
               %creamos objeto simbolico
                l1=DH(1,2);
                l2=DH(2,3);
                l3=DH(4,2);
                l5=DH(6,2);
                a1=DH(1,3);
                a2=DH(3,3);
                syms q1 q2 q3 q4 q5 q6 l1 l2 l3 l5 a1 a2 real
                dh=[0 l1 a1 pi/2 0 
                    0 0  l2   0    0
                    0 0  a2  pi/2 0
                    0 l3 0   pi/2 0 
                    0 0  0  -pi/2 0
                    0 l5 0    0    0];
                q=[q1 q2 q3 q4 q5 q6];
                R = SerialLink(dh);
                J =simplify( R.jacob0(q));
                disp(J);
                Jw=simplify(R.jacobn(q));
                disp(Jw);
                Jq = J(1:6,:);
                DetsymJ = simplify(det(Jq));
                disp(DetsymJ);
           case 2
               punto=true;
               while (punto==true)
                disp('1.Posici�n inicial q0');
                disp('2.Posici�n singular horizontal');
                disp('3.Posici�n singular vertical');
                pos=input('');
                   switch pos
                       case 1
                        q0=[0.3309    2.4763   -1.1892    2.1270    0.5639   -2.1385];
                        Jr=IRB7600.jacob0(q0); %Matriz Jacobiana de q0
                        DetJ= det(Jr);
                        disp('El determinante para una posci�n inicial (q0) es:'),disp(DetJ);
                        disp('Su rango es:'),disp(rank(Jr));
                        disp('La matriz Jacobiana Jq0 es:'),disp(Jr);
                        disp('La INVERSA de la matriz Jacobiana Jq0 es:'),disp(inv(Jr));
                        %figure (1)
                        IRB7600.plot3d(q0,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                        IRB7600.vellipse(q0,'fillcolor','b','edgecolor','w','alpha',0.5);

                        % Calculo deretminante para puntos singulares
                       case 2
                        qsingular1=[(90*pi)/180 (1*pi)/180 (90*pi)/180 (173*pi)/180 (0*pi)/180  (0*pi)/180]; %horizontal estirado
                        Js1 =IRB7600.jacob0(qsingular1);  %Matriz Jacobiana de qsingular1
                        DetJs1 =det(Js1);
                        disp('El determinante para una poscion horizontal singular es:'),disp(DetJs1);
                        disp('Su rango es:'),disp(rank(Js1));
                        disp('La matriz Jacobiana Js1 es'),disp(Js1);
                        %figure (2)
                        IRB7600.plot3d(qsingular1,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                        IRB7600.vellipse(qsingular1,'fillcolor','b','edgecolor','w','alpha',0.5);
                        
                       case 3
                        qsingular2=[(90*pi)/180 (90*pi)/180 (90*pi)/180 (0*pi)/180 (0*pi)/180  (0*pi)/180];  %Vertical estirado hacia arriba
                        Js2 =IRB7600.jacob0(qsingular2);  %Matriz Jacobiana de qsingular2
                        DetJs2 =det(Js2);
                        disp('El determinante para una posci�n vertical singular es:'),disp(DetJs2);
                        disp('Su rango es:'),disp(rank(Js2));
                        disp('La matriz Jacobiana Js2 es:'),disp(Js2);
                        %figure (3)
                        IRB7600.plot3d(qsingular2,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                        IRB7600.vellipse(qsingular2,'fillcolor','b','edgecolor','w','alpha',0.5);
                        
                       case 0
                        punto=false;
                        %si q4 y q6 estan alineadas el rango de la matriz Rank(J)
                        %es 5 porque una depende de la otra y la configuracion de
                        %ese punto seria un punto singular. 

                        %Elipse de manipulabilidad traslacional
                   end
               end
               
       end  
%==========================================================================       
 %%Planificacion de trayectorias
    case 5
        
        q0=[0.3309    2.4763   -1.1892    2.1270    0.5639   -2.1385];

        qf=[(8.53*pi)/180 (59.4*pi)/180 (26.9*pi)/180 0 (86.2*pi)/180  (8.53*pi)/180
            0.1489    1.0397    0.4164   -0.0000    1.4561    0.1489
            0.2450    0.9902    0.5391   -0.0000    1.5294    0.2450]; %posicion para sacar herramienta
        dt=0:0.1:4;
             
        %--------------------------------------------
        %    COORDENADAS DEL ESPACIO DE TRABAJO
        %--------------------------------------------

        %   HERRAMIENTA 1
        Ph10=[0.7 -1 1.55];
        qh10=[ -0.9601    1.8131   -0.4194    0.0000    1.3937   -0.9601];
        Ph11=[0.7 -1 1.5];
        qh11=[-0.9601    1.8108   -0.4645   -0.0000    1.3463   -0.9601];
        Ph12=[0.7 -0.9 1.5]; %posicion para retirar herramienta
        qh12=[-0.9291    1.8611   -0.5015    0.0000    1.3596   -0.9291];

        %   HERRAMIENTA 2
        Ph20=[1.7 -1 1.55]; %antes de agarrar herramienta
        qh20=[-0.5317    1.0918    0.3858   -0.0003    1.4774   -0.5317];      
        Ph21=[1.7 -1 1.5]; %posicion de herramienta
        qh21=[-0.5317    1.0938    0.3360   -0.0003    1.4296   -0.5317];
        Ph22=[1.7 -0.9 1.5]; %posicion para retirar herramienta
        qh22=[-0.5036    1.1264    0.2888   -0.0005    1.4149   -0.5035];
        %   ENTRADA HORNO
        %        EH=[2.5 0 1.3];
        %        qEH=[-0.0000   -0.1924    2.3543    0.0000    0.5912   -0.0000];
        %       % Tnew=troty(pi/2)*transl(0,0,2.5)*transl(-1.3,0,0)*trotz(pi); %Transformada de la entrada al horno
        %   POSICIONAMIENTO ENTRADA de HORNO
        PEH0=[1.2 0 1.3]; %Antes de entrar al horno
        qEH0=[ -0.0779    1.9862   -0.9904    2.9751    0.6028   -3.0536];
        qEH0pi=[ -0.0779    1.9862   -0.9904    2.9751    0.6028   -3.0536-(pi/2)];
        PEH1=[2.5 0 1.3]; %para entrar al horno
        qEH1=[-0.0008    0.7434    0.4777    3.1187    0.3470   -3.1257];
        qEH1pi=[-0.0008    0.7434    0.4777    3.1187    0.3470   -3.1257-(pi/2)];
        PEH2=[2.5 0 1.2]; %posicion para apoyar crisoles en el horno
        qEH2=[-0.0008    0.7267    0.4430    3.1221    0.3984   -3.1290];
        PEH3=[2.2 0 1.2];
        qEH3=[-0.0000    0.9946   -0.0460    3.1415    0.6222   -3.1416];
        PEH3=[2 0 1.2]; %posicion para retirar herramienta del hono y no tocar crisoles
        PEH4=[1 0 1.2];
        qEH4=[ -0.0000    1.1570   -0.2973    3.1415    0.7111   -3.1416];

        qEH5=[0.0000    2.1952   -1.2162   3.1416    0.5918    -3.1416];

        %   POSICION 7x6 FILAS DE CRISOLES  
        Pc10=[1 1 1.3];      %(posici�n antes de entrar a la mesa)
        %       qc10=[0.6435    1.6406   -0.7680   -1.1216   -1.0932   -0.8092];
        qc10=[0.6435    1.6406   -0.7680    2.0200    1.0932   -2.3324];

        Pc11=[1 2.3 1.3];   %Para agarrar crisoles
        %          qc11=[1.0793    0.8731    0.2565    2.2179    0.6520   -2.3465];
        qc11=[ 1.1170    0.7073    0.5406    2.1474    0.5504   -2.1898];
        Pc12=[1 2.3 1.45]; %para levantar bandeja concrisoles
        qc12=[1.1168    0.7267    0.6434    1.9831    0.5013   -1.9274];

        qc1=[qc10
        qc11];

        % Posicion de CADA UNA de las Filas de Crisoles
        % La i viene de input y la o de output
        % POSICION ANTES DE ENTRADA
        Pci0(1,:)=[1 0.4395 1.3];
        qci0(1,:)=[0.5302    2.0806   -1.0660   -2.3005    0.7461    2.4561];
        Pci0(2,:)=[1 0.2637 1.3];
        qci0(2,:)=[ 0.3381    2.1886   -1.1126   -2.5043    0.5913    2.5905];
        Pci0(3,:)=[1 0.087 1.3];
        qci0(3,:)=[0.1155    2.2509   -1.1351   -2.8835    0.4684    2.9102];
        Pci0(4,:)=[1 -0.087 1.3];
        qci0(4,:)=[ -0.1155    2.2509   -1.1351    2.8835    0.4684   -2.9102];
        Pci0(5,:)=[1 -0.2637 1.3];
        qci0(5,:)=[-0.3381    2.1886   -1.1126    2.5043    0.5913   -2.5905];
        Pci0(6,:)=[1 -0.4395 1.3];
        qci0(6,:)=[-0.5301    2.0801   -1.0667    2.3051    0.7493   -2.4601];
        Pci0(7,:)=[1 -0.6153 1.3];
        qci0(7,:)=[-0.6871    1.9470   -0.9968    2.1874    0.8906   -2.4158];
        
        %ENTRADA
        
        Pci(1,:)=[2.5 0.4395 1.3];
        qci(1,:)=[0.1929    0.6933    0.5656   -2.5747    0.3651    2.6051];
        Pci(2,:)=[2.5 0.2637 1.3];
        qci(2,:)=[0.1167    0.7256    0.5083    -2.8009    0.3558    2.8207];
        Pci(3,:)=[2.5 0.087 1.3];
        qci(3,:)=[0.0386    0.7413    0.4806   -3.0290    0.3509    3.0358];
        Pci(4,:)=[2.5 -0.087 1.3];
        qci(4,:)=[-0.0386    0.7413    0.4806    3.0290    0.3509   -3.0358];
        Pci(5,:)=[2.5 -0.2637 1.3];
        qci(5,:)=[-0.1167    0.7256    0.5083    2.8009    0.3558   -2.8207];
        Pci(6,:)=[2.5 -0.4395 1.3];
        qci(6,:)=[-0.1929    0.6933    0.5656    2.5747    0.3651   -2.6051];
        Pci(7,:)=[2.5 -0.6153 1.3];
        qci(7,:)=[-0.2669    0.6420    0.6578    2.3455    0.3781   -2.3821];
        
        %GIRAR PI/2
        
        Pc_pi(1,:)=[2.5 0.4395 1.3];
        qc_pi(1,:)=[0.1929    0.6933    0.5656   -2.5747    0.3651    2.6051+(pi/2)];
        Pc_pi(2,:)=[2.5 0.2637 1.3];
        qc_pi(2,:)=[0.1167    0.7256    0.5083    -2.8009    0.3558   2.8207+(pi/2)];
        Pc_pi(3,:)=[2.5 0.087 1.3];
        qc_pi(3,:)=[0.0386    0.7413    0.4806   -3.0290    0.3509    3.0358+(pi/2)];
        Pc_pi(4,:)=[2.5 -0.087 1.3];
        qc_pi(4,:)=[-0.0386    0.7413    0.4806    3.0290    0.3509   -3.0358+(pi/2)];
        Pc_pi(5,:)=[2.5 -0.2637 1.3];
        qc_pi(5,:)=[-0.1167    0.7256    0.5083    2.8009    0.3558   -2.8207+(pi/2)];
        Pc_pi(6,:)=[2.5 -0.4395 1.3];
        qc_pi(6,:)=[-0.1929    0.6933    0.5656    2.5747    0.3651   -2.6051+(pi/2)];
        Pc_pi(7,:)=[2.5 -0.6153 1.3];
        qc_pi(7,:)=[-0.2669    0.6420    0.6578    2.3455    0.3781   -2.3821+(pi/2)];

        %ELEVANDO CRISOLES
        Pco(1,:)=[2.5 0.4395 1.45];
        qco(1,:)=[0.1921    0.7120    0.6742   2.3388    0.2580    -2.3504];
        Pco(2,:)=[2.5 0.2637 1.45];
        qco(2,:)=[0.1159    0.7479    0.6113   2.6564    0.2349    -2.6626];
        Pco(3,:)=[2.5 0.087 1.45];
        qco(3,:)=[0.0379    0.7652    0.5814   3.0006    0.2237    -2.9988];
        Pco(4,:)=[2.5 -0.087 1.45];
        qco(4,:)=[-0.0393    0.7650    0.5816    2.9375    0.2262   -2.9476];
        Pco(5,:)=[2.5 -0.2637 1.45];
        qco(5,:)=[-0.1173    0.7474    0.6120    2.6050    0.2420   -2.6225];
        Pco(6,:)=[2.5 -0.4395 1.45];
        qco(6,:)=[ -0.1934    0.7112    0.6755    2.3037    0.2683   -2.3260];
        Pco(7,:)=[2.5 -0.6153 1.45];
        qco(7,:)=[-0.2674    0.6524    0.7801    2.0243    0.3029   -2.0464]; 

        %SALIDA
        Pco1(1,:)=[2 0.4395 1.45];
        qco1(1,:)=[0.2461    1.2099   -0.1136   2.6390    0.5302    -2.6988];
        Pco1(2,:)=[2 0.2637 1.45];
        qco1(2,:)=[0.1496    1.2404   -0.1551   2.8293    0.5062    -2.8664];
        Pco1(3,:)=[2 0.087 1.45];
        qco1(3,:)=[ 0.0497    1.2558   -0.1758   3.0365    0.4930    -3.0489];
        Pco1(4,:)=[2 -0.087 1.45];
        qco1(4,:)=[-0.0497    1.2558   -0.1758    3.0365    0.4930   -3.0489];
        Pco1(5,:)=[2 -0.2637 1.45];
        qco1(5,:)=[-0.1496    1.2404   -0.1551    2.8292    0.5062   -2.8663];
        Pco1(6,:)=[2 -0.4395 1.45];
        qco1(6,:)=[-0.2461    1.2099   -0.1136    2.6390    0.5302   -2.6988];
        Pco1(7,:)=[1 -0.6153 1.45];
        qco1(7,:)=[-0.6871    2.0193   -0.8867    2.0481    0.7953   -2.2071]; 

        %PEH4=[1 0 1.2];
        %Punto de salida para fila de crisol
        %Po0=[2.5 0 1.45];
        
        qo0=[-0.0007    0.7672    0.5779    3.1093    0.2233   -3.1152];

        %POSICION 1 Fila COLADA de Crisol 
        Pc20=[0.1 1 1.3];
        qc20=[ 1.4330    2.2145   -1.1316    2.7959    0.4688   -2.8374-(pi/2)]; %rotado -90grd para luego hacer el giro para el agarre
        Pc21=[0.1 2.3 1.3];
        qc21=[1.5201    0.9427    0.1425    2.9961    0.4788   -3.0247-(pi/2)]; 
        qc22=[1.5201    0.9427    0.1425    2.9961    0.4788   -3.0247]; %gira para agarrar fila
        Pc23=[0.1 2.3 1.45];
        qc23=[1.5201    0.9694    0.1922    2.9706    0.4034   -2.9966];
        qc20pi=[ 1.4330    2.2145   -1.1316    2.7959    0.4688   -2.8374];
        qc2=[qc20
        qc21
        qc22];
        %   POSICION 2 Fila COLADA de Crisol
        Pc30=[-0.2 1 1.3];  %posicionamiento
        qc30=[1.7290    1.6782   -0.7409   2.9021    0.6405    -2.9354];
        Pc31=[-0.2 2.3 1.45]; 
        qc31=[1.6661    0.9752    0.2350   2.9161    0.3623    -2.9170];
        qc32=[1.6661    0.9752    0.2350   2.9161    0.3623    -2.9170-pi/1.5]; %gira 120� para hacer la colada

%========================================================================== 
rutina=true;
while rutina

    disp('ELIJA RUTINA DEL IRB7600');
    disp('1.Agarrar herramienta 1');
    disp('2.Agarrar herramienta 2');
    disp('3.Dejar herramienta 1');
    disp('4.Dejar herramienta 2');
    disp('5.Agarrar crisoles 7x6 en mesa');
    disp('6.Demo filas de crisoles en HORNO');
    disp('7.DEJAR en el Horno');
    disp('8.SACAR del Horno crisoles 7x6');
    disp('9.SACAR del Horno crisoles fila (elegir fila)');
    disp('10.Hacer colada');
    disp('11.DEJAR Crisoles 7x6 en mesa');
    disp('0.Fin de operacion');

    rutina=input('');
        switch rutina
            case 1 %Agarrar herramienta 1
                close all;
                IRB7600.tool = transl(0,0,0)*trotx(0);
                [Q,Qd,Qdd]=jtraj(q0,qh10,dt);
                
                %Q = jtraj(q0,qh10,dt);
                [Q1,Q1d,Q1dd] = jtraj(qh10,qh11,dt);
                %IRB7600.plot(Q);
                hold on
                %IRB7600.plot(Q1)

                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                hold on
                Q2=jtraj(qh11,qh12,dt);
                %IRB7600.plot(Q2);
                %IRB7600.plot(Q3);
                Q6=[Q;Q1;Q2;Q3];
     
                IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                
                disp('Acople exitoso');
                
                figure (2)
                subplot(3,1,1);
                qplot(Q);
                title('Posicion antes de agarrar herramienta');
                subplot(3,1,2);
                qplot(Qd);
                subplot(3,1,3);
                qplot(Qdd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q1);
                title('Agarrando herramienta');
                subplot(3,1,2);
                qplot(Q1d);
                subplot(3,1,3);
                qplot(Q1dd);
              
                
                
            case 2  %Agarrar herramienta 2
                close all;
                IRB7600.tool = transl(0,0,0)*trotx(0);
                [Q,Qd,Qdd] = jtraj(q0,qh20,dt);
                [Q1,Q1d,Q1dd] = jtraj(qh20,qh21,dt);
%                 IRB7600.plot(Q);
                hold on
%                 IRB7600.plot(Q1)
%                 disp('Acople exitoso');
                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                hold on
                Q2=jtraj(qh21,qh22,dt);
                %IRB7600.plot(Q2);
                Q3=jtraj(qh22,q0,dt);
                %IRB7600.plot(Q3);
                Q6=[Q;Q1;Q2;Q3];
                IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                %Ploteamos posicion velocidad y aceleraciones de las articulaciones en los puntos mas importantes              
                figure (2)
                subplot(3,1,1);
                qplot(Q);
                title('Posicion antes de agarrar herramienta');
                subplot(3,1,2);
                qplot(Qd);
                subplot(3,1,3);
                qplot(Qdd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q1);
                title('Agarrando herramienta');
                subplot(3,1,2);
                qplot(Q1d);
                subplot(3,1,3);
                qplot(Q1dd);
                
            case 3 %dejar herramienta 1
                close all;
                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                [Q,Qd,Qdd] = jtraj(q0,qh10,dt);
                [Q1,Q1d,Q1dd] = jtraj(qh10,qh11,dt);
                %IRB7600.plot(Q);
                hold on
                %IRB7600.plot(Q1)
                %disp('Desacople exitoso');
                IRB7600.tool = transl(0,0,0)*trotx(0);
                hold on
                Q2=jtraj(qh11,qh12,dt);
                %IRB7600.plot(Q2);
                Q3=jtraj(qh12,q0,dt);
                %IRB7600.plot(Q3);
                Q6=[Q;Q1;Q2;Q3];
                IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                disp('Desacople exitoso');
                 
                figure (2)
                subplot(3,1,1);
                qplot(Q);
                title('Posicion antes de DEJAR herramienta');
                subplot(3,1,2);
                qplot(Qd);
                subplot(3,1,3);
                qplot(Qdd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q1);
                title('Dejando herramienta');
                subplot(3,1,2);
                qplot(Q1d);
                subplot(3,1,3);
                qplot(Q1dd);
                
                
            case 4 %dejar herramienta 2
                close all;
                IRB7600.tool = transl(0,0,0)*trotx(0);
                [Q,Qd,Qdd] = jtraj(q0,qh20,dt);
                [Q1,Q1d,Q1dd] = jtraj(qh20,qh21,dt);
                %IRB7600.plot(Q);
                hold on
                %IRB7600.plot(Q1)
                %disp('Desacople exitoso');
                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                hold on
                Q2=jtraj(qh21,qh22,dt);
                %IRB7600.plot(Q2);
                Q3=jtraj(qh22,q0,dt);
                %IRB7600.plot(Q3);
                Q6=[Q;Q1;Q2;Q3];
                IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                disp('Desacople exitoso');
                
                 figure (2)
                subplot(3,1,1);
                qplot(Q);
                title('Posicion antes de agarrar herramienta');
                subplot(3,1,2);
                qplot(Qd);
                subplot(3,1,3);
                qplot(Qdd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q1);
                title('Dejando herramienta');
                subplot(3,1,2);
                qplot(Q1d);
                subplot(3,1,3);
                qplot(Q1dd);
                 
            case 5  %Agarrar crisoles 7x6 en mesa
                close all;
                 IRB7600.tool = transl(0,0,1.3)*trotx(0);
                Q = jtraj(q0,qc10,dt);
                %IRB7600.plot(Q);
                Q1 = jtraj(qc10,qc11,dt);
                hold on
                %IRB7600.plot(Q1)
                [Q2,Q2d,Q2dd]=jtraj(qc11,qc12,dt);
                %IRB7600.plot(Q2);
                [Q3,Q3d,Q3dd]=jtraj(qc12,qc10,dt);
                %IRB7600.plot(Q3);
                Q4=jtraj(qc10,q0,dt);
                %IRB7600.plot(Q4);
                Q6=[Q;Q1;Q2;Q3;Q4];
                IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                
                figure (2)
                subplot(3,1,1);
                qplot(Q2);
                title('Levantando Crisoles');
                subplot(3,1,2);
                qplot(Q2d);
                subplot(3,1,3);
                qplot(Q2dd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q3);
                title('Retirando Crisoles de mesa');
                subplot(3,1,2);
                qplot(Q3d);
                subplot(3,1,3);
                qplot(Q3dd);
                
            case 6 %Agarrar 1 fila crisol en mes
                close all;
                n=1;
                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                Q = jtraj(q0,qEH0,dt);
                Q1 = jtraj(qEH0,qEH1pi,dt);
                hold on
                [Q2,Q2d,Q2dd]=jtraj(qEH1pi,qci0(n,:),dt);
                [Q3,Q3d,Q3dd]=jtraj(qci0(n,:),qci(n,:),dt);
                [Q4,Q4d,Q4dd]=jtraj(qci(n,:),qci0(n,:),dt);
                [Q5,Q5d,Q5dd]=jtraj(qci0(n,:),qci0(n+1,:),dt);
                
                n=n+1; %de 2 a 3
                [Q6,Q6d,Q6dd]=jtraj(qci0(n,:),qci(n,:),dt);
                [Q7,Q7d,Q7dd]=jtraj(qci(n,:),qci0(n,:),dt);
                [Q8,Q8d,Q8dd]=jtraj(qci0(n,:),qci0(n+1,:),dt);
                
                n=n+1; %de 3 a 4
                [Q9,Q9d,Q9dd]=jtraj(qci0(n,:),qci(n,:),dt);
                [Q10,Q10d,Q10dd]=jtraj(qci(n,:),qci0(n,:),dt);
                [Q11,Q11d,Q11dd]=jtraj(qci0(n,:),qci0(n+1,:),dt);
                
                n=n+1; %de 4 a 5
                [Q12,Q12d,Q12dd]=jtraj(qci0(n,:),qci(n,:),dt);
                [Q13,Q13d,Q13dd]=jtraj(qci(n,:),qci0(n,:),dt);
                [Q14,Q14d,Q14dd]=jtraj(qci0(n,:),qci0(n+1,:),dt);
                
                n=n+1; %de 5 a 6
                [Q15,Q15d,Q15dd]=jtraj(qci0(n,:),qci(n,:),dt);
                [Q16,Q16d,Q16dd]=jtraj(qci(n,:),qci0(n,:),dt);
                [Q17,Q17d,Q17dd]=jtraj(qci0(n,:),qci0(n+1,:),dt);
                
                n=n+1; %de 6 a 7
                [Q18,Q18d,Q18dd]=jtraj(qci0(n,:),qci(n,:),dt);
                [Q19,Q19d,Q19dd]=jtraj(qci(n,:),qci0(n,:),dt);
                [Q20,Q20d,Q20dd]=jtraj(qci0(n,:),qci0(n+1,:),dt);
                
                Q21=jtraj(qci0(7,:),qo0,dt);
                Q22=jtraj(qo0,q0,dt);
                Qx=[Q;Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8;Q9;Q10;Q11;Q12;Q13;Q14;Q15;Q16;Q17;Q18;Q19;Q20;Q21];
                IRB7600.plot3d(Qx,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                              
                
            case 7 %DEJAR en el horno
                close all;
                 IRB7600.tool = transl(0,0,1.3)*trotx(0);
                Q = jtraj(q0,qEH0,dt);
                %                 IRB7600.plot(Q);
                [Q1,Q1d,Q1dd ]= jtraj(qEH0,qEH1,dt);
                hold on
                %                 IRB7600.plot(Q1)
                [Q2,Q2d,Q2dd]=jtraj(qEH1,qEH2,dt);
                %                 IRB7600.plot(Q2);
                %                 disp('Crisoles en el horno');
                [Q3,Q3d,Q3dd]=jtraj(qEH2,qEH3,dt);
                %                 IRB7600.plot(Q3);
                Q4=jtraj(qEH3,qEH4,dt);
                %                 IRB7600.plot(Q4);
                Q5=jtraj(qEH4,q0,dt);
                %IRB7600.plot(Q5);
                Q6=[Q1;Q2;Q3;Q4;Q5];
                IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                disp('Crisoles en el horno');

                figure (2)
                subplot(3,1,1);
                qplot(Q1);
                title('Ingresando al horno');
                subplot(3,1,2);
                qplot(Q1d);
                subplot(3,1,3);
                qplot(Q1dd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q2);
                title('Apoyando crisoles');
                subplot(3,1,2);
                qplot(Q2d);
                subplot(3,1,3);
                qplot(Q2dd);
                
                
                figure (4)
                subplot(3,1,1);
                qplot(Q3);
                title('Retirando crisoles del horno');
                subplot(3,1,2);
                qplot(Q3d);
                subplot(3,1,3);
                qplot(Q3dd);
                

            case 8 %SACAR del Horno crisoles 7x6 (herramienta 1)
                close all;
                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                Q = jtraj(q0,qEH0,dt);
                %                 IRB7600.plot(Q);
                [Q1,Q1d,Q1dd] = jtraj(qEH0,qEH2,dt);
                hold on
                %                 IRB7600.plot(Q1)
                [Q2,Q2d,Q2dd]=jtraj(qEH2,qEH1,dt); %entro en un z mas chico (EH2) y luego levanto(EH1)
                %                 IRB7600.plot(Q2);
                %                 disp('Sacando crisoles del horno');
                 [Q3,Q3d,Q3dd]=jtraj(qEH1,qEH0,dt);
                %                 IRB7600.plot(Q3);
                Q4=jtraj(qEH0,q0,dt);
                %                 IRB7600.plot(Q4);
                Q6=[Q;Q1;Q2;Q3;Q4];
                IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                disp('Crisoles Fuera del horno');
                
                 figure (2)
                subplot(3,1,1);
                qplot(Q1);
                title('Ingresando al horno');
                subplot(3,1,2);
                qplot(Q1d);
                subplot(3,1,3);
                qplot(Q1dd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q2);
                title('Levantando crisoles');
                subplot(3,1,2);
                qplot(Q2d);
                subplot(3,1,3);
                qplot(Q2dd);


                figure (4)
                subplot(3,1,1);
                qplot(Q3);
                title('Retirando crisoles del horno');
                subplot(3,1,2);
                qplot(Q3d);
                subplot(3,1,3);
                qplot(Q3dd);

                case 9 %SACAR del Horno crisoles fila

                close all;


                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                Q1 = jtraj(q0,qEH0,dt);
                %                 Q1 = jtraj(qEH0,qEH0pi,dt);
                disp('Que fila desea sacar?');
                disp('4 5 6 o 7'); %%POR AHORA EL 1 2 y 3 no FUNCIONAN
                fila=input('');
            switch (fila)
                case 1
                    n=1;    
                    [Q2,Q2d,Q2dd]=jtraj(qEH0,qci0(n,:),dt);
                    [Q3,Q3d,Q3dd]=jtraj(qci0(n,:),qci(n,:),dt);
                    [Q4,Q4d,Q4dd]=jtraj(qci(n,:),qc_pi(n,:),dt);
                    [Q5,Q5d,Q5dd]=jtraj(qc_pi(n,:),qco(n,:),dt);
                    [Q6,Q6d,Q6dd]=jtraj(qco(n,:),qco1(n,:),dt);
                    [Q7,Q7d,Q7dd]=jtraj(qco1(n,:),qEH4,dt);
                    Q8=jtraj(qEH4,q0,dt);
                    Qx=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
                    IRB7600.plot3d(Qx,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
            
                case 2
                    n=2;    
                    [Q2,Q2d,Q2dd]=jtraj(qEH0,qci0(n,:),dt);
                    [Q3,Q3d,Q3dd]=jtraj(qci0(n,:),qci(n,:),dt);
                    [Q4,Q4d,Q4dd]=jtraj(qci(n,:),qc_pi(n,:),dt);
                    [Q5,Q5d,Q5dd]=jtraj(qc_pi(n,:),qco(n,:),dt);
                    [Q6,Q6d,Q6dd]=jtraj(qco(n,:),qco1(n,:),dt);
                    [Q7,Q7d,Q7dd]=jtraj(qco1(n,:),qEH4,dt);
                    Q8=jtraj(qEH4,q0,dt);
                    Qx=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
                    IRB7600.plot3d(Qx,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                case 3
                    n=3;    
                    [Q2,Q2d,Q2dd]=jtraj(qEH0,qci0(n,:),dt);
                    [Q3,Q3d,Q3dd]=jtraj(qci0(n,:),qci(n,:),dt);
                    [Q4,Q4d,Q4dd]=jtraj(qci(n,:),qc_pi(n,:),dt);
                    [Q5,Q5d,Q5dd]=jtraj(qc_pi(n,:),qco(n,:),dt);
                    [Q6,Q6d,Q6dd]=jtraj(qco(n,:),qco1(n,:),dt);
                    [Q7,Q7d,Q7dd]=jtraj(qco1(n,:),qEH4,dt);
                    Q8=jtraj(qEH4,q0,dt);
                    Qx=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
                    IRB7600.plot3d(Qx,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                case 4
                    n=4;    
                    [Q2,Q2d,Q2dd]=jtraj(qEH0,qci0(n,:),dt);
                    [Q3,Q3d,Q3dd]=jtraj(qci0(n,:),qci(n,:),dt);
                    [Q4,Q4d,Q4dd]=jtraj(qci(n,:),qc_pi(n,:),dt);
                    [Q5,Q5d,Q5dd]=jtraj(qc_pi(n,:),qco(n,:),dt);
                    [Q6,Q6d,Q6dd]=jtraj(qco(n,:),qco1(n,:),dt);
                    [Q7,Q7d,Q7dd]=jtraj(qco1(n,:),qEH4,dt);
                    Q8=jtraj(qEH4,q0,dt);
                    Qx=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
                    IRB7600.plot3d(Qx,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                case 5
                    n=5;    
                    [Q2,Q2d,Q2dd]=jtraj(qEH0,qci0(n,:),dt);
                    [Q3,Q3d,Q3dd]=jtraj(qci0(n,:),qci(n,:),dt);
                    [Q4,Q4d,Q4dd]=jtraj(qci(n,:),qc_pi(n,:),dt);
                    [Q5,Q5d,Q5dd]=jtraj(qc_pi(n,:),qco(n,:),dt);
                    [Q6,Q6d,Q6dd]=jtraj(qco(n,:),qco1(n,:),dt);
                    [Q7,Q7d,Q7dd]=jtraj(qco1(n,:),qEH4,dt);
                    Q8=jtraj(qEH4,q0,dt);
                    Qx=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
                    IRB7600.plot3d(Qx,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                case 6
                    n=6;    
                    [Q2,Q2d,Q2dd]=jtraj(qEH0,qci0(n,:),dt);
                    [Q3,Q3d,Q3dd]=jtraj(qci0(n,:),qci(n,:),dt);
                    [Q4,Q4d,Q4dd]=jtraj(qci(n,:),qc_pi(n,:),dt);
                    [Q5,Q5d,Q5dd]=jtraj(qc_pi(n,:),qco(n,:),dt);
                    [Q6,Q6d,Q6dd]=jtraj(qco(n,:),qco1(n,:),dt);
                    [Q7,Q7d,Q7dd]=jtraj(qco1(n,:),qEH4,dt);
                    Q8=jtraj(qEH4,q0,dt);
                    Qx=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
                    IRB7600.plot3d(Qx,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                case 7
                    n=7;    
                    [Q2,Q2d,Q2dd]=jtraj(qEH0,qci0(n,:),dt);
                    [Q3,Q3d,Q3dd]=jtraj(qci0(n,:),qci(n,:),dt);
                    [Q4,Q4d,Q4dd]=jtraj(qci(n,:),qc_pi(n,:),dt);
                    [Q5,Q5d,Q5dd]=jtraj(qc_pi(n,:),qco(n,:),dt);
                    [Q6,Q6d,Q6dd]=jtraj(qco(n,:),qco1(n,:),dt);
                    [Q7,Q7d,Q7dd]=jtraj(qco1(n,:),qEH4,dt);
                    Q8=jtraj(qEH4,q0,dt);
                    Qx=[Q1;Q2;Q3;Q4;Q5;Q6;Q7;Q8];
                    IRB7600.plot3d(Qx,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                 
               end 
                
            case 10 %Hacer colada
                    close all;
                    IRB7600.tool = transl(0,0,1.3)*trotx(0);
                    Q = jtraj(q0,qc30,dt);
                    %                 IRB7600.plot(Q);
                    [Q1,Q1d,Q1dd] = jtraj(qc30,qc31,dt);
                    hold on
                    %                 IRB7600.plot(Q1)
                    [Q2,Q2d,Q2dd]=jtraj(qc31,qc32,dt);
                    %                 IRB7600.plot(Q2);
                    %                 disp('Colada realizada');
                    Q3=jtraj(qc32,qc31,dt);
                    %                 IRB7600.plot(Q3);
                    Q4=jtraj(qc31,qc30,dt);
                    %                 IRB7600.plot(Q4);
                    Q5=jtraj(qc30,q0,dt);
                    %                 IRB7600.plot(Q5);
                    Q6=[Q1;Q2;Q3;Q4;Q5];
                    IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                    disp('Colada realizada');
                
                figure (2)
                subplot(3,1,1);
                qplot(Q1);
                title('Girando herramienta');
                subplot(3,1,2);
                qplot(Q1d);
                subplot(3,1,3);
                qplot(Q1dd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q2);
                title('Realizando Colada');
                subplot(3,1,2);
                qplot(Q2d);
                subplot(3,1,3);
                qplot(Q2dd);
                
                
            case 11 %DEJR Crisoles 7x6 en mesa
                close all;
                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                Q = jtraj(q0,qc10,dt);
                %                 IRB7600.plot(Q);
                [Q1,Q1d,Q1dd] = jtraj(qc10,qc12,dt);
                hold on
                %                 IRB7600.plot(Q1)
                [Q2,Q2d,Q2dd]=jtraj(qc12,qc11,dt);
                %                 IRB7600.plot(Q2);
                %                 disp('Crisoles en mesa');
                [Q3,Q3d,Q3dd]=jtraj(qc11,qc10,dt);
                %                 IRB7600.plot(Q3);
                Q4=jtraj(qc10,q0,dt);
                %                 IRB7600.plot(Q4);
                Q6=[Q;Q1;Q2;Q3;Q4];
                IRB7600.plot3d(Q6,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                
                disp('Crisoles en mesa');       
                
                figure (2)
                subplot(3,1,1);
                qplot(Q1);
                title('Posicionando en la mesa');
                subplot(3,1,2);
                qplot(Q1d);
                subplot(3,1,3);
                qplot(Q1dd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q2);
                title('Apoyando crisoles en mesa');
                subplot(3,1,2);
                qplot(Q2d);
                subplot(3,1,3);
                qplot(Q2dd);
                
                
                figure (4)
                subplot(3,1,1);
                qplot(Q3);
                title('Retirando herramienta 1 de mesa');
                subplot(3,1,2);
                qplot(Q3d);
                subplot(3,1,3);
                qplot(Q3dd);
                
                
                
            case 12 %dejar Crisoles fila en mesa
                close all;
                IRB7600.tool = transl(0,0,1.3)*trotx(0);
                Q = jtraj(q0,qc20pi,dt);
              %  IRB7600.plot3d(Q,'path',pwd);
                Q1 = jtraj(qc20pi,qc23,dt);
                hold on;
               % IRB7600.plot3d(Q1,'path',pwd);
                [Q2,Q2d,Q2dd]=jtraj(qc23,qc22,dt);
               % IRB7600.plot3d(Q2,'path',pwd);
                [Q3,Q3d,Q3dd]=jtraj(qc22,qc21,dt);
              %  IRB7600.plot3d(Q3,'path',pwd);
                [Q4,Q4d,Q4dd]=jtraj(qc21,qc20,dt);
               % IRB7600.plot3d(Q4,'path',pwd);
               Q5=jtraj(qc20,q0,dt);
                Q5=[Q;Q1;Q2;Q3;Q4;Q5];
                IRB7600.plot3d(Q5,'path',pwd,'workspace',[-6,6,-6,6,-1.2,4],'view',[-45,20]);
                disp('Crisoles en mesa');
                
                figure (2)
                subplot(3,1,1);
                qplot(Q2);
                title('Apoyando Crisoles en mesa');
                subplot(3,1,2);
                qplot(Q2d);
                subplot(3,1,3);
                qplot(Q2dd);
                
                figure (3)
                subplot(3,1,1);
                qplot(Q2);
                title('Girando Herramienta 2');
                subplot(3,1,2);
                qplot(Q2d);
                subplot(3,1,3);
                qplot(Q2dd);
                
                
                figure (4)
                subplot(3,1,1);
                qplot(Q3);
                title('Retirando herramienta 2 de mesa');
                subplot(3,1,2);
                qplot(Q3d);
                subplot(3,1,3);
                qplot(Q3dd);
        end
    end
end
disp('Finalizado');
