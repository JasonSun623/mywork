import socket
import json
import ast
host = "192.168.0.201"
port = 8081
a = 'c6001e'
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host, port))
s.sendall(a)
data = s.recv(1024).strip()
str_dic_data = (data.strip('\0'))
dict_data = ast.literal_eval(str_dic_data)
print(dict_data),type(dict_data)
print(dict_data['status'],type(dict_data['status']))

s.close()
