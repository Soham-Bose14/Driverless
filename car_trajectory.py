import matplotlib.pyplot as plt
import pandas as pd
import car_info

dataframe = pd.read_csv("/home/sohambose/ws/waypoints.csv")
X = list(dataframe.iloc[:,0])
Y = list(dataframe.iloc[:,1])

dataframe2 = pd.read_csv("/home/sohambose/ws/new_waypoints.csv")
X2 = list(dataframe2.iloc[:,0])
Y2 = list(dataframe2.iloc[:,1])

dataframe3 = pd.read_csv("/home/sohambose/ws/blue_cone_pos.csv")
Xb = list(dataframe3.iloc[:,0])
Yb = list(dataframe3.iloc[:,1])

dataframe4 = pd.read_csv("/home/sohambose/ws/yellow_cone_pos.csv")
Xy = list(dataframe4.iloc[:,0])
Yy = list(dataframe4.iloc[:,1])

'''print(f'Blue x: {car_info.Xblue}')
print(f'Yellow x: {car_info.Xyellow}')
print(f'Blue y: {car_info.Yblue}')
print(f'Yellow y: {car_info.Yyellow}')'''


'''fig, ax = plt.subplots(facecolor=(.18, .31, .31))'''
plt.scatter(X,Y,color = 'black')
plt.scatter(X2,Y2, color='red')
plt.scatter(Xb,Yb, color = 'blue')
plt.scatter(Xy,Yy, color = 'yellow')
'''for i in range(len(X)):
    plt.text(i,)'''
plt.show()
