import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

import seaborn as sns
sns.set_theme(style="darkgrid")


df = pd.read_csv('wall-climb-results.csv')

df = df.replace(0, 0.02)


narrow = df[~df.Type.str.contains("w")]
wide = df[~df.Type.str.contains("n")]

#df df.replace('?', np.NaN)


narrow_05 = narrow[narrow['Alt'] == 0.5]
narrow_10 = narrow[narrow['Alt'] == 1]
narrow_15 = narrow[narrow['Alt'] == 1.5]
narrow_20 = narrow[narrow['Alt'] == 2]

wide_05 = wide[wide['Alt'] == 0.5]
wide_10 = wide[wide['Alt'] == 1]
wide_15 = wide[wide['Alt'] == 1.5]
wide_20 = wide[wide['Alt'] == 2]


narrow1 = narrow[narrow['Agents'] == 1]
narrow5 = narrow[narrow['Agents'] == 5]
narrow8 = narrow[narrow['Agents'] == 8]
narrow10 = narrow[narrow['Agents'] == 10]
narrow20 = narrow[narrow['Agents'] == 20]
narrow50 = narrow[narrow['Agents'] == 50]
narrow100 = narrow[narrow['Agents'] == 100]

wide1 = wide[wide['Agents'] == 1]
wide5 = wide[wide['Agents'] == 5]
wide8 = wide[wide['Agents'] == 8]
wide10 = wide[wide['Agents'] == 10]
wide20 = wide[wide['Agents'] == 20]
wide50 = wide[wide['Agents'] == 50]
wide100 = wide[wide['Agents'] == 100]

print(narrow_10)

plt.figure()
plt.plot(narrow_05["Percent"],narrow_05["Agents"], label = "Narrow, 0.5")
plt.plot(wide_05["Percent"],wide_05["Agents"], label = "Wide, 0.5")
plt.plot(narrow_10["Percent"],narrow_10["Agents"], label = "Narrow, 1.0")
plt.plot(wide_10["Percent"],wide_10["Agents"], label = "Wide, 1.0")
plt.plot(narrow_15["Percent"],narrow_15["Agents"], label = "Narrow, 1.5")
plt.plot(wide_15["Percent"],wide_15["Agents"], label = "Wide, 1.5")
plt.plot(narrow_20["Percent"],narrow_20["Agents"], label = "Narrow, 2.0")
plt.plot(wide_20["Percent"],wide_20["Agents"], label = "Wide, 2.0")

plt.xlabel('Wall Climb Success Rate')
plt.ylabel('Number of Agents')
plt.legend()



fig, ax = plt.subplots()
width = 5.35  # the width of the bars
labels = ['1', '5', '8', '10', '20', '50', '100']
rects1 = ax.bar(narrow_05["Agents"],narrow_05["Percent"], width, label='Men')
#rects2 = ax.bar(narrow_10["Agents"],narrow_10["Percent"], width, label='Women')

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Scores')
ax.set_title('Scores by group and gender')
x = np.arange(len(labels))  # the label locations
ax.set_xticks(x, labels)
ax.legend()

ax.bar_label(rects1, padding=3)
#ax.bar_label(rects2, padding=3)



plt.figure()

X = ['1', '5', '8', '10', '20', '50', '100']
  
X_axis = np.arange(len(X))
  
plt.bar(X_axis - 0.25, narrow_05["Percent"], .1, label = '0.5 - Narrow', color = "#42e9ff")
plt.bar(X_axis - 0.05, narrow_10["Percent"], .1,  label = '1.0 - Narrow', color = '#259afa')
plt.bar(X_axis + 0.15, narrow_15["Percent"], .1,  label = '1.5 - Narrow' , color = '#2b4fff')
plt.bar(X_axis + 0.35, narrow_20["Percent"], .1,  label = '2.0 - Narrow', color = '#0e00a3')

plt.bar(X_axis - 0.35, wide_05["Percent"], .1, label = '0.5 - Wide', color = "#FFE010")
plt.bar(X_axis - 0.15, wide_10["Percent"], .1,  label = '1.0 - Wide', color = '#FFA12C')
plt.bar(X_axis + 0.05, wide_15["Percent"], .1,  label = '1.5 - Wide' , color = '#FE612C')
plt.bar(X_axis + 0.25, wide_20["Percent"], .1,  label = '2.0 - Wide', color = '#F11D28')
  
plt.xticks(X_axis, X)
plt.xlabel("Number of Agents")
plt.ylabel("Wall Climbing Success Rate")
#plt.title("Number of Students in each group")
plt.legend()

'''
plt.figure()
X = ['1', '5', '8', '10', '20', '50', '100']
  
X_axis = np.arange(len(X))
  
plt.bar(X_axis - 0.25, narrow_05["Percent"], .2, label = '0.5 - Narrow', color = "#42e9ff")
plt.bar(X_axis - 0.05, narrow_10["Percent"], .2,  label = '1.0 - Narrow', color = '#259afa')
plt.bar(X_axis + 0.15, narrow_15["Percent"], .2,  label = '1.5 - Narrow' , color = '#2b4fff')
plt.bar(X_axis + 0.35, narrow_20["Percent"], .2,  label = '2.0 - Narrow', color = '#0e00a3')
plt.legend()

print(narrow1)
'''

fig = plt.figure(figsize=(15,4))
X = ['0.5', '1.0','1.5','2.0']
X_axis = np.arange(len(X))
plt.bar(X_axis - 0.15, narrow1["Percent"], .05, label = '1', color = "#9dfbff" )
plt.bar(X_axis - 0.1, narrow5["Percent"], .05,  label = '5', color = "#25f3fb")
plt.bar(X_axis - 0.05, narrow8["Percent"], .05,  label = '8', color = '#09dbf9')
plt.bar(X_axis + 0.00, narrow10["Percent"], .05,  label = '10', color = '#09b1f9' )
plt.bar(X_axis + 0.05, narrow20["Percent"], .05, label = '20', color = '#0073f7' )
plt.bar(X_axis + 0.1, narrow50["Percent"], .05,  label = '50', color = '#0646ff')
plt.bar(X_axis + 0.15, narrow100["Percent"], .05,  label = '100', color = '#0014b1' )

plt.xticks(X_axis, X)
plt.xlabel("Altitude Setpoints")
plt.ylabel("Average Wall Climbing \n Success Rate (5 Trials)")
import matplotlib.ticker as mtick
plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(1))
plt.legend()


fig = plt.figure(figsize=(15,4))
X = ['0.5', '1.0','1.5','2.0']
X_axis = np.arange(len(X))
plt.bar(X_axis - 0.15, wide1["Percent"], .05, label = '1', color = "#f4fe4b" )
plt.bar(X_axis - 0.1, wide5["Percent"], .05,  label = '5', color = "#fce900")
plt.bar(X_axis - 0.05, wide8["Percent"], .05,  label = '8', color = '#fcc800')
plt.bar(X_axis + 0.00, wide10["Percent"], .05,  label = '10', color = '#fcaa00' )
plt.bar(X_axis + 0.05, wide20["Percent"], .05, label = '20', color = '#fc6b00' )
plt.bar(X_axis + 0.1, wide50["Percent"], .05,  label = '50', color = '#fc0000')
plt.bar(X_axis + 0.15, wide100["Percent"], .05,  label = '100', color = '#af0000' )

plt.xticks(X_axis, X)
plt.xlabel("Altitude Setpoints")
plt.ylabel("Average Wall Climbing \n Success Rate (5 Trials)")
import matplotlib.ticker as mtick
plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(1))
plt.legend()

'''
fig, ax = plt.subplots()
X = ['1', '5', '8', '10', '20', '50', '100']
X_axis = np.arange(len(X))

y = np.vstack([narrow_05["Percent"], narrow_10["Percent"], narrow_15["Percent"],narrow_20["Percent"]])
#ax.stackplot(x, y)

plt.figure()
plt.plot(X_axis, narrow_05["Percent"])
plt.plot(X_axis, narrow_10["Percent"])
plt.plot(X_axis, narrow_15["Percent"])
plt.plot(X_axis, narrow_20["Percent"])
plt.xlabel("Number of Agents")
plt.ylabel("Wall Climbing Success Rate")
#plt.title("Number of Students in each group")
plt.legend()


plt.figure()
X = ['0.5', '1.0','1.5','2.0']
X_axis = np.arange(len(X))
plt.plot(X_axis, narrow1["Percent"], .1, label = '1', )
plt.plot(X_axis, narrow5["Percent"], .1,  label = '5',)
plt.plot(X_axis, narrow8["Percent"], .1,  label = '8' )
plt.plot(X_axis, narrow10["Percent"], .1,  label = '10')
plt.plot(X_axis, narrow20["Percent"], .1, label = '20', )
plt.plot(X_axis, narrow50["Percent"], .1,  label = '50',)
plt.plot(X_axis, narrow100["Percent"], .1,  label = '100' )
plt.xticks(X_axis, X)
plt.xlabel("Altitude Setpoints")
plt.ylabel("Wall Climbing Success Rate")
plt.legend()
'''
plt.show()
