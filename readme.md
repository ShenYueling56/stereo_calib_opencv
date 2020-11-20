### 生成file_left.txt和file_right.txt用于分别存储左右图像的地址, 
```
# 参数为存储标定图片的地址
python scripts/gen_file_list/gen_file_list.py /media/shenyl/Elements/sweeper/sweeper_data_example
```

### 进行双目相机内参和外参标定
```
# -d=存放标定图片的地址 -w=标定板横向内格点数 -h=标定板纵向内格点数 -s标定板格子边长(单位mm) -show=(True:显示双目校正后水平线用于判断矫正结果好坏；False：不显示水平线)
-d=/media/shenyl/Elements/sweeper/sweeper_data_example/ -show=True
```