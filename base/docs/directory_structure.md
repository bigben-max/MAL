# 代码目录结构

主要参考ethz-asl实验室相关代码

|  一级目录   |  说明  |
|  ----  | ----  |
| cmake/  | 放必要的 *.cmake文件 |
| config/  | 程序配置文件 |
| launch/  | ros node的启动 *.launch文件 |
| docs/  | 放代码相关的说明文档 |
| inc/  | 代码头文件 |
| node/  | ros node 文件, 主进程main, 按照 xxxxx_node.cc命名 |
| src/  | 代码源文件.cc, 里面的层级和inc一一对应 |
| proto/  | *.proto文件 |
| test/  | 单元测试的*_test.cc文件 |
| CMakeLists.txt  | cmake文件 |
| package.xml  | package.xml文件 |

