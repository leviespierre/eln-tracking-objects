from tkinter import *
# these two imports are important
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
import numpy as np

from ot_functions import *
 
#%%
continuePlotting = True   
a=1
def data_points(sample):
    [rightClusters, leftClusters]=get_ClustersList(sample)
    posX=[]
    posY=[]
    clusType=[]
    for clus in range(len(rightClusters)):
        posY.append(rightClusters[clus].f_DistX)
        posX.append(-(rightClusters[clus].f_DistY))
        clusType.append(rightClusters[clus].s_ClusterKinematicID)
    for clus in range(len(leftClusters)):
        posY.append(leftClusters[clus].f_DistX)
        posX.append(-(leftClusters[clus].f_DistY))
        clusType.append(leftClusters[clus].s_ClusterKinematicID)
    posX=np.asarray(posX)
    posY=np.asarray(posY)           
    clusType=np.asarray(clusType)
    clusTypeC = np.where(clusType=='Static','c',np.where(clusType=="Ambig", 'g',np.where(clusType=="Static-Ambig", 'r','m')) )
    clusTypeFig = np.where(clusType=='Static','+',np.where(clusType=="Ambig", '*',np.where(clusType=="Static-Ambig", '.','p')) )
    print(np.shape(posX),np.shape(posY),np.shape(clusTypeFig))
    return [posX, posY,clusTypeC,clusTypeFig]
def data_object(sample):
    if sample>2:
        [rightObjects, leftObjects] = get_objectsList(sample)
        OposX=[]
        OposY=[]
        for obj in range(len(rightObjects.list_40TrackedObjects)):
            OposY.append(rightObjects.list_40TrackedObjects[obj].f_DistX)
            OposX.append(-rightObjects.list_40TrackedObjects[obj].f_DistY)
    
        for obj in range(len(leftObjects.list_40TrackedObjects)):
            OposY.append(leftObjects.list_40TrackedObjects[obj].f_DistX)
            OposX.append(-leftObjects.list_40TrackedObjects[obj].f_DistY)
        return [OposX,OposY]

def app():
    # initialise a window.
    root = Tk()
    root.config(background='white')
    root.geometry("1000x700")
    
    lab = Label(root, text="Live Plotting", bg = 'white').pack()
    
    fig = Figure()
    
    ax = fig.add_subplot(111)
    ax.set_xlabel("X axis")
    ax.set_ylabel("Y axis")
    ax.set_xlim(-100, 100)
    ax.set_ylim(-100,100)
    ax.grid()

    graph = FigureCanvasTkAgg(fig, master=root)
    graph.get_tk_widget().pack(side="top",fill='both',expand=True)
    
    

    def change_state():
        global continuePlotting
        if continuePlotting == True:
            continuePlotting = False
        else:
            continuePlotting = True
 
    def plotter():
        global a
        if continuePlotting:
            ax.cla()
            ax.grid()            
            [x,y, co,figu] = data_points(a)
            
            print(np.shape(x),np.shape(y),np.shape(co))
            if np.shape(x)>(0,):
                 ax.scatter(x, y,c=co, marker='X')
                 if a>2:
                     [ox,oy]=data_object(a)
                     ax.scatter(ox,oy,c='b', marker='o')
                 ax.set_xlim(-100, 100)
                 ax.set_ylim(-100,100)
                 graph.draw()
           
            a += 1
            print (a)
        root.after(1, plotter)

    def gui_handler():
        change_state()
        plotter()
    start = Button(root, text="Start", command=plotter)
    b = Button(root, text="Puse/Continue", command=change_state, bg="red", fg="white")
    b.pack()
    start.pack()
    
    root.mainloop()
    root.quit()
 
if __name__ == '__main__':
    app()
#%%