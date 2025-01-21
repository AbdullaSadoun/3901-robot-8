import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.dates import date2num
from datetime import datetime

# Data for the Gantt chart
data = {
    "Task": ["Project Planning", "System Design", "Component Selection", "Software Development", 
             "Hardware Integration", "Testing and Validation", "Final Testing", 
             "Meeting 1", "Meeting 2", 
             "Meeting 3", "Meeting 4", 
             "Meeting 5", "Meeting 6", 
             "Meeting 7"],
    "Start Date": ["04/07/24", "06/07/24", "09/07/24", "11/07/24", "15/07/24", "16/07/24", "18/07/24", 
                   "04/07/24", "06/07/24", "09/07/24", "11/07/24", "15/07/24", "16/07/24", "18/07/24"],
#    "End Date": ["06/07/24", "09/07/24", "11/07/24", "15/07/24", "16/07/24", "18/07/24", "19/07/24", 
#                 "04/07/24", "06/07/24", "09/07/24", "11/07/24", "15/07/24", "16/07/24", "18/07/24"]
    "End Date": ["06/07/24", "09/07/24", "11/07/24", "15/07/24", "16/07/24", "18/07/24", "19/07/24", 
                 "05/07/24", "07/07/24", "10/07/24", "12/07/24", "16/07/24", "17/07/24", "19/07/24"]
}

# Convert data to DataFrame
df = pd.DataFrame(data)

# Convert dates to datetime
df['Start Date'] = pd.to_datetime(df['Start Date'], format='%d/%m/%y')
df['End Date'] = pd.to_datetime(df['End Date'], format='%d/%m/%y')

# Plotting the Gantt chart
fig, ax = plt.subplots(figsize=(10, 8))

#for i, task in enumerate(df['Task']):
 #   start = df.loc[i, 'Start Date']
  #  end = df.loc[i, 'End Date']
   # #ax.barh(task, (end - start).days, left=date2num(start), align='center', color='red')
    #ax.text(date2num(start) + (end - start).days / 2, i, task, ha='center', va='center', color='black')
#    if "Meeting" in task:
 #       color = 'blue'
  #  else:
   #     color = 'red'
    #ax.barh(task, (end - start).days, left=date2num(start), align='center', color=color)


# Plot non-meeting tasks first
for i, task in enumerate(df['Task']):
    if "Meeting" not in task:
        start = df.loc[i, 'Start Date']
        end = df.loc[i, 'End Date']
        ax.barh(task, (end - start).days, left=date2num(start), align='center', color='red')

# Plot meeting tasks on top
for i, task in enumerate(df['Task']):
    if "Meeting" in task:
        start = df.loc[i, 'Start Date']
        end = df.loc[i, 'End Date']
        #ax.barh(task, (end - start).days, left=date2num(start), align='center', color='blue')
        ax.barh(task, 1, left=date2num(start), align='center', color='blue')

ax.set_xlabel('Date')
ax.set_ylabel('Task')
ax.xaxis_date()
ax.invert_yaxis()
plt.title('ECED3901 Gantt Chart')
#plt.grid(axis='x', linestyle='--', alpha=0.6)
plt.grid(axis='both', linestyle='--', alpha=0.6)

plt.savefig('gantt_chart.png') # saves tge the gantt chart to a file

#plt.tight_layout() # Adjust the plot to fit the labels
plt.show() 