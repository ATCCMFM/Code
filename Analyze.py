import matplotlib.pyplot as plt 
#import atexit
import os
from datetime import datetime
import math

count=1
ODreading=0

#__________________________________
list_of_files = [

'2019Y11M12D_15H21M08','2019Y11M13D_21H04M13','2019Y11M15D_19H45M19','2019Y11M16D_16H57M37'
]

start_times_labels = [
'tubido','lowst metabo','turbido','3rd metabo'
]

NewestExp=str(list_of_files[len(list_of_files)-1])
previous_endtime=0
title='pbad+(M9_0.4%_Glucos_1%_(10X)_and_0%_Arabinose'
timeTicksIntervel=2
PlotODsetTo=0
PlotMotors=0
#__________________________________
x_ticks = 3600

fileDir = os.path.join(os.path.dirname(os.path.realpath('__file__')),os.pardir)
NewestExpDir = fileDir + '/Experiments/' + NewestExp
plotDir = NewestExpDir + '/plots'
if not os.path.exists(plotDir): # make the plot directory if it does not exist
    os.makedirs(plotDir)

first_start_time=datetime.strptime(list_of_files[0], "%YY%mM%dD_%HH%MM%S")
start_time_after_1st_start=0



tick_lbls = []
exp_time = []
TempSt = []
TCouple = []

Sens0St = []
Sens0Op = []
Exci0St = []
Exci0Op = []
Exci0Rf = []
FluoroTarget0 = []
PMT0rds = []

Sens1St = []
Sens1Op = []
Exci1St = []
Exci1Op = []
Exci1Rf = []
FluoroTarget1=[]
PMT1rds = []


ODSET = []
ODrds = []
ODLED = []
ODCalculated=[]
ODSETCalculated=[]
ODTime=[]
ODSETTime=[]
Stir = []
Heater = []
PumpWN = []
PumpWS = []
PumpEN = []
PumpES = []
ODLEDrf = []
LevelLED = []
Level = []
LevelLEDrf = []
ODratio = []
Levelratio = []
start_times=[]
PumpWNWSRatio=[]


for file_name in list_of_files:

    start_time = datetime.strptime(file_name, '%YY%mM%dD_%HH%MM%S')
    start_time_after_1st_start=(start_time-first_start_time).total_seconds() 
    start_times.append(start_time_after_1st_start)

    file_name_with_path = fileDir + '/Experiments/' + file_name + '/raw_data/' + file_name + '.txt'
    f = open(file_name_with_path,'rU')

    next(f)
    for line in f:
      data = line.split("\t")

      if len(data) == 33:
        exptime=float(data[0])+start_time_after_1st_start
        exp_time.append(exptime)
        TempSt.append(float(data[1]))
        TCouple.append(float(data[2]))
        Sens0St.append(float(data[3]))
        Sens0Op.append(float(data[4]))
        Exci0St.append(float(data[5]))
        Exci0Op.append(float(data[6]))
        Exci0Rf.append(float(data[7]))
        FluoroTarget0.append(float(data[8]))
        PMT0rds.append(float(data[9]))

        Sens1St.append(float(data[10]))
        Sens1Op.append(float(data[11]))
        Exci1St.append(float(data[12]))
        Exci1Op.append(float(data[13]))
        Exci1Rf.append(float(data[14]))
        FluoroTarget1.append(float(data[15]))
        PMT1rds.append(float(data[16]))

        ODSET.append(float(data[17]))

        ODrds.append(float(data[18]))
        ODLEDrf.append(float(data[19]))
        ODLED.append(float(data[20])*0.2)

        Level.append(float(data[21]))
        LevelLEDrf.append(float(data[22]))
        LevelLED.append(float(data[23])*0.1)

        Stir.append(float(data[24])*0.4)
        Heater.append(float(data[25])*0.6)

        PumpWN.append(float(data[26]))
        PumpWS.append(float(data[27]))
        PumpEN.append(float(data[28]))
        PumpES.append(float(data[29]))
        PumpWNWSRatio.append(float(data[30]))


        ODratio.append(float(data[31]))
        Levelratio.append(float(data[32]))

        if float(data[18]) != 0 and float(data[19]) !=0 :
            ODCalculated.append(math.log((float(data[19])*0.7)/float(data[18])))
            ODTime.append(exptime)
            ODReading= float(data[18])
        
        if float(data[17]) != 0 and float(data[19]) !=0 :
            ODSETCalculated.append(math.log((float(data[19])*0.7)/float(data[17])))
            ODSETTime.append(exptime)


    f.close()
    previous_endtime=max(exp_time)
    #print (start_times_labels[count])
    
    #print (ODCalculated[-1])
    #print (ODReading)

    #print ("")

    #count=count+1

print('Ploting...')
#------------------------------------------------------------------------

maxlen=previous_endtime-1

fig = plt.figure(facecolor='white',figsize=(10,15))
fig.suptitle(title, fontsize=10)

grid = plt.GridSpec(23, 1, wspace=0.05, hspace=0.06,bottom=0.08,top=0.95,left= 0.1,right=0.9)
ax1 = fig.add_subplot(grid[:3, 0])#OD ratio
ax2 = fig.add_subplot(grid[3:5, 0])# OD set, OD Read, OD ref
ax3 = fig.add_subplot(grid[5:9, 0])# PMT0read
ax4 = fig.add_subplot(grid[9, 0])#PMT0SenseSet,PMT0SensePin
ax5 = fig.add_subplot(grid[10, 0])#Excite0set,Excite0Pin Excite0Ref
ax6 = fig.add_subplot(grid[11:15, 0])# PMT1read
ax7 = fig.add_subplot(grid[15, 0])#PMT1SenseSet,PMT1SensePin
ax8 = fig.add_subplot(grid[16, 0])#Excite1set,Excite1Pin Excite1Ref
ax9 = fig.add_subplot(grid[17:19, 0])# TempSet, TempRead
ax10 = fig.add_subplot(grid[19, 0])#LevelRatio
ax11 = fig.add_subplot(grid[20, 0])#Level, Level LEDrf
ax12 = fig.add_subplot(grid[21, 0])#Heater,Stir,Pumpin,Pumpout
ax15 = fig.add_subplot(grid[22, 0])


ax13 = ax1.twiny()
ax14 = ax15.twiny()


ax1.set_ylim(0, 1100)
ax1.set_xlim(0, maxlen)
ax2.set_ylim(-0.1, 1.8)
ax2.set_xlim(0, maxlen)
ax2.set_yticks([0,1])

ax3.set_ylim(0, 1100)
ax3.set_xlim(0, maxlen)
ax4.set_xlim(0, maxlen)
ax4.set_ylim(0, 1.5)
ax4.set_yticks([0.5,1.1])
ax4.set_ylabel("0.5-1.1"+"\n"+"Volt", color="black")
ax5.set_xlim(0, maxlen)
ax5.set_ylim(0, 1100)
ax5.set_yticks([500,1000])

ax6.set_ylim(0, 1100)
ax6.set_xlim(0, maxlen)
ax7.set_xlim(0, maxlen)
ax7.set_ylim(0, 1.5)
ax7.set_yticks([0.5,1.1])
ax7.set_ylabel("0.5-1.1"+"\n"+"Volt", color="black")
ax8.set_xlim(0, maxlen)
ax8.set_ylim(0, 1100)
ax8.set_yticks([500,1000])


ax9.set_ylim(20, 46)
ax9.set_yticks(range(31,45, 3))
ax9.set_xlim(0, maxlen)
ax9.set_ylabel("Degree", color="r")
ax9.tick_params(axis="y", labelcolor="r")
ax10.set_ylim(0, 1)
ax10.set_xlim(0, maxlen)
ax11.set_xlim(0, maxlen)
ax11.set_ylim(0, 1100)
ax11.set_yticks([500,1000])
ax12.set_xlim(0, maxlen)
ax12.set_ylim(0, 1.9)
ax13.set_xlim(0, maxlen)
ax14.set_xlim(0, maxlen)
ax15.set_xlim(0, maxlen)
ax15.set_ylim(0, 1)


#ax12.set_yticklabels([])
ax12.set_ylabel("ON/OFF", color="black")


plot1 = ax1.scatter(exp_time, ODrds, s=2, marker='o', linewidth='0',facecolor='orange', label='ODrds')
plot2 = ax1.scatter(exp_time, ODLEDrf, s=3, marker='o', linewidth='0', facecolor='black', label='ODLEDrf')
plot3 = ax1.scatter(exp_time, ODSET, s=3, marker='o', linewidth='0', facecolor='darkred', label='ODSET')
ax1.legend(loc='upper right',ncol=1,fontsize = 'xx-small', frameon=False)  

plot4 = ax2.scatter(ODTime, ODCalculated, s=3, marker='o', linewidth='0', facecolor='orange', label='ODCalculated')
plot42 = ax2.scatter(ODSETTime, ODSETCalculated, s=3, marker='o', linewidth='0', facecolor='darkred', label='ODSETCalculated')

ax2.legend(loc='upper right',ncol=3,fontsize = 'xx-small',frameon=False)  

plot5 = ax3.scatter(exp_time, PMT0rds, s=3, marker='o', linewidth='0', facecolor='green', label='PMT0read')
plot6 = ax3.scatter(exp_time, FluoroTarget0, s=3, marker='o', linewidth='0', facecolor='greenyellow', label='PMT0Set')
ax3.legend(loc='upper right',fontsize = 'xx-small',frameon=False)  

plot7 = ax4.scatter(exp_time, Sens0St, s=3, marker='o', linewidth='0', facecolor='darkgreen', label='PMT0SenseSet')
plot8 = ax4.scatter(exp_time, Sens0Op, s=3, marker='o', linewidth='0', facecolor='greenyellow', label='PMT0SensePin')
ax4.legend(loc='upper right',ncol=2,fontsize = 'xx-small',frameon=False)
plot9 = ax5.scatter(exp_time, Exci0St, s=3, marker='o', linewidth='0', facecolor='darkblue', label='Excite0Set')
plot10 = ax5.scatter(exp_time, Exci0Op, s=3, marker='o', linewidth='0', facecolor='cyan', label='Excite0Pin')
plot11 = ax5.scatter(exp_time, Exci0Rf, s=3, marker='o', linewidth='0', facecolor='blue', label='Excite0rf')
ax5.legend(loc='upper right',ncol=3,fontsize = 'xx-small',frameon=False)  

plot12 = ax6.scatter(exp_time, PMT1rds, s=3, marker='o', linewidth='0', facecolor='red', label='PMT1read')
plot13 = ax6.scatter(exp_time, FluoroTarget1, s=3, marker='o', linewidth='0', facecolor='pink', label='PMT1Set')

ax6.legend(loc='upper right',fontsize = 'xx-small',frameon=False)  

plot14 = ax7.scatter(exp_time, Sens1St, s=3, marker='o', linewidth='0', facecolor='darkred', label='PMT1SenseSet')
plot15 = ax7.scatter(exp_time, Sens1Op, s=3, marker='o', linewidth='0', facecolor='pink', label='PMT1SensePin')
ax7.legend(loc='upper right',ncol=2,fontsize = 'xx-small',frameon=False)
plot16 = ax8.scatter(exp_time, Exci1St, s=3, marker='o', linewidth='0', facecolor='orange', label='Excite1Set')
plot17 = ax8.scatter(exp_time, Exci1Op, s=3, marker='o', linewidth='0', facecolor='brown', label='Excite1Pin')
plot18 = ax8.scatter(exp_time, Exci1Rf, s=3, marker='o', linewidth='0', facecolor='yellow', label='Excite1rf')
ax8.legend(loc='upper right',ncol=3,fontsize = 'xx-small',frameon=False)  

plot19 = ax9.scatter(exp_time, TempSt, s=3, marker='o', linewidth='0', facecolor='darkred', label='TempSet')
plot20 = ax9.scatter(exp_time, TCouple, s=3, marker='o', linewidth='0', facecolor='red', label='TempRead')
ax9.legend(loc='upper right',ncol=2,fontsize = 'xx-small',frameon=False)  

plot21 = ax10.scatter(exp_time, Levelratio, s=3, marker='o', linewidth='0', facecolor='blue', label='LevelRatio')
ax10.legend(loc='upper right',ncol=1,fontsize = 'xx-small',frameon=False)  

plot22 = ax11.scatter(exp_time, Level, s=3, marker='o', linewidth='0', facecolor='blue', label='Level')
plot23 = ax11.scatter(exp_time, LevelLEDrf, s=3, marker='o', linewidth='0', facecolor='black', label='LevelLEDrf')
ax11.legend(loc='upper right',ncol=2, fontsize = 'xx-small',frameon=False)  

plot24 = ax12.scatter(exp_time, ODLED, s=3, marker='o', linewidth='0', facecolor='orange', label='ODLED')
plot25 = ax12.scatter(exp_time, LevelLED, s=3, marker='o', linewidth='0', facecolor='darkred', label='LevelLED')
plot26 = ax12.scatter(exp_time, Heater, s=3, marker='o', linewidth='0', facecolor='red', label='Heater')
plot27 = ax12.scatter(exp_time, Stir, s=3, marker='o', linewidth='0', facecolor='greenyellow', label='Stir')
plot28 = ax12.scatter(exp_time, PumpWN, s=3, marker='o', linewidth='0', facecolor='blue', label='PumpIn')
plot29 = ax12.scatter(exp_time, PumpEN, s=3, marker='o', linewidth='0', facecolor='darkblue', label='PumpOut')
ax12.legend(loc='upper right',ncol=6, fontsize = 'xx-small',frameon=False)  

plot30 = ax15.scatter(exp_time, PumpWNWSRatio, s=3, marker='o', linewidth='0', facecolor='darkblue', label='PumpWNWSRatio')


for j in range (0,len(start_times)):
    ax1.axvline(x=start_times[j])
    ax2.axvline(x=start_times[j])
    ax3.axvline(x=start_times[j])
    ax4.axvline(x=start_times[j])
    ax5.axvline(x=start_times[j])
    ax6.axvline(x=start_times[j])
    ax7.axvline(x=start_times[j])
    ax8.axvline(x=start_times[j])
    ax9.axvline(x=start_times[j])
    ax10.axvline(x=start_times[j])
    ax11.axvline(x=start_times[j])
    ax12.axvline(x=start_times[j])
    ax15.axvline(x=start_times[j])



#x_interval = maxlen/10

if maxlen < 3600: # less than 1 hours
    timescale = "Time (minutes)"
    scale = 60
elif maxlen < 36000: # less than 10 hours
    timescale = "Time (hours)"
    scale = 3600
elif maxlen < 864000: # less than 10 days
    timescale = "Time (10 hours)"
    scale = 36000
else: # ten days or more
    timescale = "Time (days)"
    scale = 86400
x_ticks_list = []
x_ticks_label_list = []
x_interval = scale

for j in range(int(maxlen/x_interval)+1):
    x_ticks_list.append(int(x_interval*j))
    x_ticks_label_list.append(int(x_interval*j/scale))


ax1.set_xticks(x_ticks_list)
ax2.set_xticks(x_ticks_list)
ax3.set_xticks(x_ticks_list)
ax4.set_xticks(x_ticks_list)
ax5.set_xticks(x_ticks_list)
ax6.set_xticks(x_ticks_list)
ax7.set_xticks(x_ticks_list)
ax8.set_xticks(x_ticks_list)
ax9.set_xticks(x_ticks_list)
ax10.set_xticks(x_ticks_list)
ax11.set_xticks(x_ticks_list)
ax12.set_xticks(x_ticks_list)

ax1.set_xticklabels([])
ax2.set_xticklabels([])
ax3.set_xticklabels([])
ax4.set_xticklabels([])
ax5.set_xticklabels([])
ax6.set_xticklabels([])
ax7.set_xticklabels([])
ax8.set_xticklabels([])
ax9.set_xticklabels([])
ax10.set_xticklabels([])
ax11.set_xticklabels([])
ax12.set_xticklabels([])

ax15.set_xticklabels(x_ticks_label_list)
ax15.set_xlabel(timescale)


ax13.set_xticks(start_times)
#ax13.set_xticklabels(start_times_labels)
ax13.set_xticklabels(start_times_labels, rotation = 45, ha="left",fontsize = 'x-small')

ax14.set_xticks(start_times)
ax14.xaxis.tick_bottom()
ax14.set_xticklabels(list_of_files, rotation = 45, ha="right",fontsize = 'x-small')


##plt.show()

plt.savefig(plotDir + '/results_' + 'total' + '.png', dpi=600, format="png") #save plot
plt.close(fig)


  