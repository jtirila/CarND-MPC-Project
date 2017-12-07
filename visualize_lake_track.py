CSVFILE = 'lake_track_waypoints.csv'
import csv
import matplotlib
# matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

def load_data():
    xs = []
    ys = []
    with open(CSVFILE, 'r') as csvfile: 
        reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        for row in reader:
            # Just a hack for this particular case to remove the csv header row
            if row[0] == 'x': 
                continue

            xs.append(float(row[0])) 
            ys.append(float(row[1])) 
    return xs, ys 



xs, ys = load_data()

print zip(xs[:10], ys[:10])

plt.scatter(xs, ys, s=2)
plt.show()
