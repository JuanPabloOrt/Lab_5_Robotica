# Lab_5_Robotica

Theylor Amaya; Juan Pablo Ortiz 2023-2

## Cinematica inversa
 Inicialmente se establece la longitud de cada eslabón del robot, todo esto para realizar la tabla DH y el tipo de movimiento de cada eslabon y asi poder creal el modelo simulado del robot

![ROBOT_SImulado](https://github.com/JuanPabloOrt/lab4_robotica_phantom/assets/144562439/3c7c519e-4470-4170-85cc-fd47d45e9990)
Este robot cuenta con 4 atriculaciones rotativas y ninguna de tipo longitudinal, ademas de la herramienta.
ESTE Permite realizar movimientos en el entorno virtual de como se comportaria el robot, y obsrvar la posicion en que quedaría ubicado segun la rotación dada al robot.

![DH](https://github.com/JuanPabloOrt/lab4_robotica_phantom/assets/144562439/07d9b675-9a88-4055-94bf-4df021bbdffe)

Esta es la matriz DH del robot

### Parámetros Denavit-Hartenberg (DH) 
<div align="center">

| $\mathbf{i}$ | $\mathbf{\theta_i}$ | $\mathbf{d_i}$ | $\mathbf{a_i}$ | $\mathbf{\alpha_i}$ |$\mathbf{offset_i}$ |
|:------------:|:-------------------:|:--------------:|:--------------:|:-------------------:|:-------------------:|
|      $1$     |         $q_1$       |      $L_1$     |       $0$      |   $-\frac{\pi}{2}$  |         $0$         |
|      $2$     |         $q_2$       |       $0$      |      $L_2$     |         $0$         |   $-\frac{\pi}{2}$  |
|      $3$     |         $q_3$       |       $0$      |      $L_3$     |         $0$         |         $0$         |
|      $4$     |         $q_4$       |       $0$      |      $L_4$     |         $0$         |         $0$         |

</div>

* L_1 = 42 mm
* L_2 = 104 mm
* L_3 = 104 mm
* L_4 = 90 mm

![robot pincher](https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/87277411-87ac-44ef-a672-a89a53429b8a)


Ahora se procede a hallar la cinematica inversa del robot, usando una funcion creada llamada ikine queda

```matlab
function q = ikine(x,y,z)
syms theta2 theta3;
l1=9.2;
l2=4.8; 
l3=10.6;
l4=10.6; 
l5=10.5;
theta1=double(vpa(atan(y/x)));
eqn1 = z == l1+l2+sin(theta2)*l3+sin(theta2+theta3)*l4;
eqn3 = y == sin(theta1)*(cos(theta2)*l3+cos(theta2+theta3)*l4+l5);
S = solve([eqn1 eqn3],[theta2 theta3],'Real',true);
theta4=-(vpa(S.theta2(1))+vpa(S.theta3(1)));
q=[theta1 vpa(S.theta2(1)) vpa(S.theta3(1)) theta4];
  end
```

Esta función nos permite hallar los angulos segun una posición XYZ que se le de al robot, establecendo dentro de la función la longitud de los eslabones como se ve en el codigo, dando asi para la posicion
y=0.01
x=30
z=7.7 

los siguientes angulos en cada eslabon


q =

         0   -0.0530   -0.5180    0.5720

Con este codigo implementado debajo de la creación del robot se da solución a la cinematica inversa del robot
```matlab

 y=0.01
 x=30
 z=7.7
 q=double(round(ikine(x,y,z),3))
 qfin=[q(1), pi/2-q(2),-q(3),-q(4)]
 view([4 30]);
 Robot_pincher.plot(qfin,'workspace',ws);
```
### Metodo geometrico

## Modelo implementado en Python

## Trazado de trayectoria del Robot con ROS y Python
