import matplotlib.pyplot as plt 
import math
import numpy as np
from pylab import *


r0 = 10     # radius of obstacle
rg = 2      #radius of goal
O = [25,25]   #position of obstacle
G = [35,40]     #position of goal
s0 = 10         #field of obstacle
sg = 50        #field of goal
b = -8     #beta
a =  0.1        #alpha

x_pos = []
y_pos = []
loc_x = []
loc_y = []
DD_obs = []
DDlog = []
DD_goal = []
colormap='jet'

radius_x = []
radius_y = []

for i in range(1,50,2):
    y0 = i
    for j in range(1,50,2):
        x0 = j
        d0 = (((x0-O[0])**2+(y0-O[1])**2)**(1/2)) 
        DD_obs.append(d0)
        dg = (((x0-G[0])**2+(y0-G[1])**2)**(1/2))
        DD_goal.append(dg)
        
        

for i in range(1,50,2):
    y0 = i
    for j in range(1,50,2):
        #conditions for obstacles
        x0 = j
        d0 = (((x0-O[0])**2+(y0-O[1])**2)**(1/2)) 
        theta = math.atan2((y0-O[1]),(x0-O[0]))
        d0log=log10(d0-min(DD_obs)+1.0)
        
        
        if(d0 < r0):
            del_x_obs = np.sign(math.cos(theta+np.pi/2))*math.inf
            del_y_obs = np.sign(math.sin(theta+np.pi/2))*math.inf
            
            
        elif(r0 <= d0 and d0 <= s0+r0 ):
            del_x_obs = b*(s0+r0-d0)*math.cos(theta+np.pi/2)
            del_y_obs = b*(s0+r0-d0)*math.sin(theta+np.pi/2)
            
            
        elif( s0+r0 < d0 ):
            del_x_obs = 0
            del_y_obs = 0
            
            
        #conditions for Goal
        dg = (((x0-G[0])**2+(y0-G[1])**2)**(1/2)) 
        theta_g = math.atan2((y0-G[1]),(x0-G[0]))
        dglog=log10(dg-min(DD_goal)+1.0)
        
        if(dg < rg):
            del_x_goal = 0
            del_y_goal = 0
            
            
        elif(rg <= dg and dg <= sg+rg ):
            del_x_goal = -a*(dg-rg)*math.cos(theta_g)
            del_y_goal = -a*(dg-rg)*math.sin(theta_g)
            
            
        elif( sg+rg < dg ):
            del_x_goal = -a*sg*math.cos(theta_g)
            del_y_goal = -a*sg*math.sin(theta_g)
            
        
        del_x = del_x_obs + del_x_goal 
        del_y = del_y_obs + del_y_goal 
        DDlog.append(d0log*dglog)
        x_pos.append(x0)
        y_pos.append(y0)
        loc_x.append(del_x)
        loc_y.append(del_y)
        
        
        
        #if((((O[0]-x0)**2+(O[1]-y0)**2)**(1/2)) == r0):
         #   radius_x.append(x0)
          #  radius_y.append(y0)


#conditions for agents
locationX = []
locationY = []
x_pos_agent = []
y_pos_agent = []
loc_x_agent = []
loc_y_agent = []

agent = [1,2]
k = 0.2
dt = 0.1
v = 1
si=0 # np.pi/3          #intial angle of bot
Rmin= 3*v
umin = -(v**2/Rmin)
umax = (v**2/Rmin)
        

locationX.append(agent[0])
locationY.append(agent[1])



for i in range(200000):
#while True:    
    #conditions for obstacles
    d_agent_obs = (((agent[0]-O[0])**2+(agent[1]-O[1])**2)**(1/2)) 
    theta_agent_obs = math.atan2((O[1]-agent[1]),(O[0]-agent[0]))
    #d0log_agent=log10(d0-min(DD_obs)+1.0)
        
        
    if(d_agent_obs < r0):
        del_x_obs_agent = -(np.sign(math.cos(theta_agent_obs+np.pi/2))*math.inf)
        del_y_obs_agent = -(np.sign(math.sin(theta_agent_obs+np.pi/2))*math.inf)
            
            
    elif( d_agent_obs >= r0 and d_agent_obs <= s0+r0 ):
        del_x_obs_agent = -b*(s0+r0-d_agent_obs)*math.cos(theta_agent_obs+np.pi/2)
        del_y_obs_agent = -b*(s0+r0-d_agent_obs)*math.sin(theta_agent_obs+np.pi/2)
            
            
    elif( d_agent_obs > s0+r0 ):
        del_x_obs_agent = 0
        del_y_obs_agent = 0
            
            
    #conditions for Goal
    d_agent_g = (((agent[0]-G[0])**2+(agent[1]-G[1])**2)**(1/2)) 
    theta_g_agent = math.atan2((G[1]-agent[1]),(G[0]-agent[0]))
    #dglog_agent=log10(dg-min(DD_goal)+1.0)
        
    if(d_agent_g < rg):
        del_x_goal_agent = 0
        del_y_goal_agent = 0
        #break
            
            
    elif( d_agent_g >= rg and d_agent_g <= sg+rg ):
        del_x_goal_agent = a*(d_agent_g-rg)*math.cos(theta_g_agent)
        del_y_goal_agent = a*(d_agent_g-rg)*math.sin(theta_g_agent)
            
            
    elif( sg+rg < d_agent_g ):
        del_x_goal_agent = a*sg*math.cos(theta_g_agent)
        del_y_goal_agent = a*sg*math.sin(theta_g_agent)
            
        
    del_x_agent = del_x_obs_agent + del_x_goal_agent 
    del_y_agent = del_y_obs_agent + del_y_goal_agent 
    #DDlog.append(d0log_agent*dglog_agent)
    #x_pos_agent.append(agent[0])
    #y_pos_agent.append(agent[1])
    #loc_x_agent.append(del_x_agent)
    #loc_y_agent.append(del_y_agent)
    

    thetad = math.atan2(del_y_agent,del_x_agent)


    u = k*(thetad-si)
    if( umin >= u ):
        u = umin
    if( u > umax ):
        u= umax
    
    #si desiaer
    si = (si + u*dt)
    
    
    #new agent position updates
    agent[0] = agent[0] + (v*math.cos(si)*dt)
    agent[1] = agent[1] + (v*math.sin(si)*dt)
    
    
    locationX.append(agent[0])
    locationY.append(agent[1])
   
    if((((agent[0]-G[0])**2+(agent[1]-G[1])**2)**(1/2))<=rg):#((rg+0.5)**2)):
       break




fig, ax = plt.subplots()
circle_obs = plt.Circle((O[0], O[1]),0.5, color='r')
circle_obs_rad = plt.Circle((O[0], O[1]),r0, color='blue')
circle_goal_rad = plt.Circle((G[0], G[1]),rg, color='red')        
circle_goal = plt.Circle((G[0], G[1]),0.5, color='blue')            
Q  = quiver(x_pos, y_pos, loc_x, loc_y, DDlog, cmap=colormap)
colorbar()
title('Potential field based on collision avoidance')       
#show()
#plt.show()
path  = plt.plot(locationX,locationY)
#rad  = plt.plot(radius_x,radius_y)
#path  = quiver(x_pos_agent, y_pos_agent, loc_x_agent, loc_y_agent, , cmap='Dark2')



#ax.add_artist(circle_obs)
#plt.axis([0,50,0,50])
ax.add_artist(circle_obs_rad)
ax.add_artist(circle_goal_rad)
ax.add_artist(circle_obs)
ax.add_artist(circle_goal)
ax.add_artist(Q)
ax.add_artist(path)
fig.show()

#fig.savefig('plotcircles.png')
