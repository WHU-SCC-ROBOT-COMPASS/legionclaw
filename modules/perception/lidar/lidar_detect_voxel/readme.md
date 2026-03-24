## 编译

```
source modules/message/ros2/install/setup.bash
cd modules/perception/lidar/lidar_detect_voxel
#此编译脚本用于编译x64平台的可执行程序，默认编译debug，如需更改为Release，则修改脚本
./scripts/build.x64.sh 
```


### 运行

```
cd modules/perception/lidar/lidar_detect_voxel/bin
./lidar_detect
```

## orin运行前先转换模型
onnx_dir为model下的rpn_centerhead.onnx路径，save_dir为保存的模型路径，记得和lidar_detect.json配置文件中rpn_engine_file_path一致
```
export PATH=/usr/src/tensorrt/bin:$PATH
trtexec --onnx=${onnx_dir} --saveEngine=${save_dir} --workspace=4096 --fp16 --outputIOFormats=fp16:chw --inputIOFormats=fp16:chw --verbose --dumpLayerInfo --dumpProfile --separateProfileRun
```