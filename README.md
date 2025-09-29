# mask03
test_ceres.cpp中迭代次数多于test_ceres1.cpp
test_ceres.cpp中将所有出现小球的帧全部提取参与计算，而test_ceres1.cpp每两帧提取一帧。
故test_Ceres1.cpp的效率较test_ceres.cpp提高，但是可能会不如其准确。
