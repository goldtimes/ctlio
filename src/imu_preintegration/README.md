# imu预积分
* vins-fusion中使用四元素表示旋转
* slam_auto_driving中使用so3表示旋转
* 预积分，同样和ekf需要估计协方差的传播，但是运用ceres或者g2o的时候会对预积分的结构做优化
# Blog
* https://blog.csdn.net/weixin_45626706/article/details/118074274
* https://blog.csdn.net/slender_1031/article/details/127548106?spm=1001.2014.3001.5502