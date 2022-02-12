import torch
c=[]
x=torch.tensor(100)
y=torch.tensor(200)
x2=torch.tensor(300)
y2=torch.tensor(400)
d=[x,y,x+y,x-y]
e=[x2,y2,x2+y2,x2-y2]
c.append(d)
c.append(e)
print(c)
#print(c[0][0])

for i in range(len(c)):
    cx=int((c[i][0]+c[i][2])/2)
    cy=int((c[i][1]+c[i][3])/2)
    print(cx)
    print(cy)

'''
[[tensor(933.), tensor(54.), tensor(982.), tensor(98.)], [
    tensor(457.), tensor(462.), tensor(517.), tensor(525.)]]

'''

