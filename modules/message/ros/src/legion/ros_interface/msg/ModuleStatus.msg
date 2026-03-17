Header   header     #  
string   name     # 发送指令的节点名称
int8   self_id     # 发送节点的ID号，参考apps描述
int32   status     #  枚举方式 sef_id+status,比如设置1为开，2为关，当前sef_id是21，则在关闭状态下反馈值是 212
Message[] messages # 需要输出的消息列表：eg:要求显示剩余里程数为100米，self_id 为21，里程数id为08，code值为2108，value值为10000，精度为cm
