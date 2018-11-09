import csv
import matplotlib as plt

path = '/home/jiongrui/PycharmProjects/El2425_Automatic_Control_Group1/'

path_csv = open(path + 'real_path.csv')
path = csv.reader(path_csv, delimiter='\n')
xlist = []
ylist = []
for row in path:
    x = csv.reader(row,delimiter = ',')
    for a in x:
        print(a)
