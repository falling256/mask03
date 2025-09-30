# mask03
test_ceres.cpp中迭代次数多于test_ceres1.cpp

test_ceres.cpp中将所有出现小球的帧全部提取参与计算，而test_ceres1.cpp每两帧提取一帧。
故test_Ceres1.cpp的效率较test_ceres.cpp提高，但是可能会不如其准确。
<img width="1081" height="239" alt="截图 2025-09-30 23-44-58" src="https://github.com/user-attachments/assets/d9fcfd6f-144a-4978-8799-6ffe19051dfd" />
<img width="1089" height="241" alt="截图 2025-09-30 23-44-00" src="https://github.com/user-attachments/assets/6c570160-6fb5-43a3-8899-36908ce99621" />
