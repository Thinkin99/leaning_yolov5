import os
 
path = "D:\\Desktop\\YOLO_PORTnoaudio\\labels\\train\\"
 
# 获取该目录下所有文件，存入列表中
fileList = os.listdir(path)
#print(fileList)
alldata = []
for i in fileList:
    with open(path+i, "r") as f:  # 打开文件
        data = f.readlines()  # 读取文件
        alldata.append(data)
        for j in data :
            #print(j)
            if  j[0] == '5' :
                print('error in:')
                print(i)
            else:
                continue
print('errors above')