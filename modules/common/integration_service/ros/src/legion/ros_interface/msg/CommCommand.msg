Headers   header     #  
string   name     # 发送指令的节点名称
int8   self_id     # 发送节点的ID号，参考apps描述
Command[] commands # 需要发送的指令，如果需要发送给多个节点，需要使用多个command
