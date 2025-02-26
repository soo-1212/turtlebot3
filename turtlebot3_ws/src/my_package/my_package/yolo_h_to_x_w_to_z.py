x1 =[ 25,-25,200, 150]
y1 = [ 0.01, -0.01,-0.01, 0.01]
x2 = [320, -320,480, 5]
y2 = [ 0.7, -0.7,-0.4, 0.4]




def cul(x1, y1, x2, y2):

    b = (y1/y2)**(1/(x1-x2))
    a = y1 / (b**x1)
    return a , b

count = 0
for i1, j1, i2, j2 in zip(x1, y1, x2, y2):
    
    count += 1
    a , b = cul(i1, j1, i2, j2)
    print(f'{count}번째 : {a}*{b}**x')
    y3 = a*b**i1
    print(y3)
    y4 = a*b**i2
    print(y4)

