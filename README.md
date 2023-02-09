# ROS2-PurePursuitControl-PathPlanning-Tracking
ROS2 , Turtlebot3 , A* ve PurePursuit kullanılarak oluşturulan Rota Bulma ve İzleme Uygulaması


![Screenshot_5](https://user-images.githubusercontent.com/87595266/205762696-91c48af3-617d-4784-a1d9-ebe66400df4c.png)

# V1.1 Sürüm Güncellemesi 09.02.2023

## Yenilikler

- Pure Pursuit algoritması optimize edildi. Artık daha iyi yol takibi yapar.
- Yol noktaları için B - Spline algoritması eklendi. Bu sayede yol yumuşatması sağlanarak rota takip optimizasyonu sağlandı.

 ![Screenshot from 2023-01-31 21-28-34](https://user-images.githubusercontent.com/87595266/217913980-c0ec9e54-0f9c-4488-8a21-2d258873a409.png)
 
 - Costmap algoritması optimize edildi.

# Nasıl Çalışır
Gerekli kurulumlar sonrasında komutu çalıştırın.  
  
`ros2 run nav_controller control`

Sonrasında rviz2 üzerinden hedef nokta belirleyin.  

# Youtube Önizleme & Kullanım Videosu  
https://youtu.be/r_2mMyaLLaI

### Notlar  
Bu paket statik ortamda tek global planla hareket için yazılmıştır.  
Dinamik ortamda global plan ve ona bağlı kalan lokal planlama paketi yakın zamanda..
