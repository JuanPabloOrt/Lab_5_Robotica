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
### Modelo geometrico 

Se crea el siguiente modelo geometricon con el cual se espera que el robot trace dentro del tablon la figura como sigue
![geometria](https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/ff2afe5f-386c-4b69-b433-5bf76b9b417a)



## Modelo implementado en Python

Con el siguiente codigo de python se ejecuto el ROS en  linux 
``` python
import rospy
import math as m
import pandas as pd
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Función de cinemática inversa
def invKine(y, x, z, o):
    arriba = 7  # altura herramienta no está escribiendo
    abajo = 4  # altura herramienta está escribiendo
    base = 6  # altura tomar la herramienta de la base
    apertura = 0  # valor de articulación 5 abierta
    cierre = 1.3  # valor de articulación 5 cerrada

    if z > 19.5 and z < 20.5:
        z = arriba
    if z > 3.5 and z < 4.5:
        z = abajo
    if o > 0.9 and o < 1.1:
        th5 = apertura
    else:
        th5 = cierre
    if x == 0:
        x = 0.01

    h = 14
    l1 = 10.6
    l2 = 10.6
    l3 = 11
    R1 = m.sqrt(pow(x, 2) + pow(y, 2))
    th1 = m.atan(y/x)
    zm = z - h
    R2 = R1 - l3
    th3 = m.acos((pow(R2, 2) + pow(zm, 2) - pow(l1, 2) - pow(l2, 2)) / (2*l1*l2))
    th2 = m.atan(R2 / zm)
    th4 = -(m.pi / 2) + abs(th2) + abs(th3)
    # if th2 > 1:
    #     th2 = 0
    return [-th1, th2, -th3, th4, th5]

# Función para leer puntos desde un archivo Excel
def read_points_from_excel(file_path):
    return pd.read_excel(file_path)

def main():
    # Inicialización del nodo ROS
    rospy.init_node('robot_joint_controller', anonymous=True)

    # Publicador para la trayectoria de las articulaciones
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    # Leer los puntos desde el archivo Excel
    points = read_points_from_excel('puntos.xlsx')

    while not rospy.is_shutdown():
        for _, row in points.iterrows():
            # Aplicar cinemática inversa a cada punto
            y, x, z = row['y'], row['x'], row['z']
            angles = invKine(y, x, z, 1)  # Asumimos 'o' como 0, ajustar según sea necesario

            # Crear un mensaje JointTrajectory
            trajectory = JointTrajectory()
            trajectory.header.stamp = rospy.Time.now()
            trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(1)
            trajectory.points.append(point)

            # Publicar el mensaje
            pub.publish(trajectory)

            # Pequeña pausa entre puntos
            rospy.sleep(1)

if __name__ == '__main__':
    main()

```
En el codigo previp se realizaron los siguientes 5 pasos pasos para lograr inicialemnte el psicionamiento y luego la indicación d elas trayectoriasdel robto Phantom.


1. Cinematica inversa y psicionamiento del cero como el tablero de trazado.
2. Lectura de los puntos de trazado establecidos en excel
3. Creacion del main donde se inicia el nodo ROS
4. Iniciación del posicionamiento y trazado de la geometria


## Trazado de trayectoria del Robot con ROS y Python
A continuación se visualiza como trazo el robot la trayectoria en el tablon mediante la conexion entre ROS y Python.

# Trayectoria arco pequeño

https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/c81e7034-71fb-4831-9152-98574972bab1

# Trayectoria arco grande

https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/f4c85bb5-e00c-430a-9335-919068369c34

# Trayectoria figura personalizada (estrella)

https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/e494c3f9-cbf8-4469-bd2d-6cfcc1532fd4

# Trayectoria figura personalizada (diamante)

https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/b668a47d-2de7-45d2-9456-6c7b7d8764d8

# Trayectoria figura personalizada (triangulo)

https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/fb12138c-9b35-4404-8a5c-cf83edefaf4b

# Trayectoria figura geometrica (inicial T)

https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/5f267530-b5c2-47ff-982a-50b0c27a4b0f

# Trayectoria figura geometrica (inicial J)

https://github.com/JuanPabloOrt/Lab_5_Robotica/assets/144562439/76b015cb-6fc6-48d1-b3eb-e770e9b9fd7f


A partir de estos videos se observa que luego de posicionada la herramienta en el robot, se logró trazar la trayectoria deseada en el espacio de trabajo designado.

## Conclusiones


Se observa que a partir de una trayectoria dada con ayuda de la ubicación de los puntos en una matriz, se puede logra que en movimientos ya sea rectos o curvilineos el robot siga esta trayectorias, ademas se observa que por medio de la metodologia de cinematica inversa es posible tambien lograr el movimiento del robot phantomx a traves del software libre ROS, siendo asi la unica necesidad en equipo de pago, disponer del robot, ya que ROS tambien permite realizar la simulacion de estos movimientos
