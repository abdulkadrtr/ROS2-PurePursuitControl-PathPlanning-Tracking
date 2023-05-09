# ROS2-PurePursuitControl-PathPlanning-Tracking

[en-readme](https://github.com/abdulkadrtr/ROS2-PurePursuitControl-PathPlanning-Tracking/blob/main/readme-en.md)


ROS2 , Turtlebot3 , A* ve PurePursuit kullanılarak oluşturulan Rota Bulma ve İzleme Uygulaması


![Screenshot_5](https://user-images.githubusercontent.com/87595266/205762696-91c48af3-617d-4784-a1d9-ebe66400df4c.png)

# V.0.1.0 Sürüm Güncellemesi | 09.02.2023

![1-min](https://user-images.githubusercontent.com/87595266/217926638-2232239a-5f35-469e-829c-a2883f835bdc.gif)



## Yenilikler

- Pure Pursuit algoritması optimize edildi. Bu sayede rota takip optimizasyonu sağlandı.
- Yol noktaları için B - Spline algoritması eklendi. Bu sayede yol yumuşatması sağlanarak rota takip optimizasyonu sağlandı.

 ![Screenshot from 2023-01-31 21-28-34](https://user-images.githubusercontent.com/87595266/217913980-c0ec9e54-0f9c-4488-8a21-2d258873a409.png)
 
 - Costmap algoritması optimize edildi.

# V.0.2.0 Sürüm Güncellemesi | 09.05.2023
![Screenshot from 2023-05-09 12-29-40](https://user-images.githubusercontent.com/87595266/237058148-8cd753df-9058-4126-ae95-c9e28d89f006.png)

- `path_follow.launch.py` dosyası eklendi. Böylece haritalama paketi ve rota takip paketi tek komutla çalıştırılabilir duruma getirildi. Ayrıca bir RVİZ2 penceresi açılarak mevcut harita ve rota görselleştirilmektedir. 
`v.0.2.0` sürümünü kullanmak için aşağıdaki kodu çalıştırın.

`ros2 launch nav_controller path_follow.launch.py`


# Nasıl Çalışır

Bir gazebo simulasyon dünyası başlatın. Örneğin;

`export TURTLEBOT3_MODEL=burger`


`ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py`



Sonrasında harita paketini çalıştırın ve haritalama işlemini gerçekleştirin.

`ros2 launch slam_toolbox online_async_launch.py`

PathPlannig-Tracking paketini çalıştırın.
  
`ros2 run nav_controller control`

Sonrasında rviz2 üzerinden hedef nokta belirleyin.  

# Youtube Önizleme & Kullanım Videosu  
https://youtu.be/r_2mMyaLLaI

## Gereklilikler

- ROS2 - Humble
- Slam Toolbox
- Turtlebot3 Paketi
- Gazebo Simulasyonu

