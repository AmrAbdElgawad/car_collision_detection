

#key----->object id from tracker  ====================  value list of tuples
# (x,y,h,w)
import numpy as np

def overlaped(trajectory):
    OverlapList=[]
    overlap=False
    for key in trajectory.keys():
        for second_key in trajectory.keys():
            
            if second_key<=key:
                continue
            
            if (2*abs(trajectory[key][-1][0]-trajectory[second_key][-1][0])< trajectory[key][-1][3]+trajectory[second_key][0][3]) ^ \
            (2*abs(trajectory[key][-1][1]-trajectory[second_key][-1][1])< trajectory[key][-1][2]+trajectory[second_key][-1][2]):
                OverlapList.append([key,second_key])
                overlap=True
    if len(OverlapList)!=0:
      frames=len(trajectory[OverlapList[0][0]])
    else:
      frames=0
    return OverlapList,frames,overlap
 

def calc_overlaped_cars_angles(trajectories,overlaped):
    '''
    inputs: trajectories ------> dictionary(obj_id:list of tuples of centers of objects ex: [(x1,y1),(x2,y2)....] ) contains trajectories of all objects in the scene
            overlaped----------> list contians the overlaped objects .
    
    outputs : the angle between overlaped objects
    
    '''

    overlaped_1=np.array(trajectories[overlaped[0]])
    overlaped_2=np.array(trajectories[overlaped[1]])
    print(overlaped_1,'\n',overlaped_2)
    overlaped_1=np.array([overlaped_1[-1][:2],overlaped_1[-2][:2]])
    overlaped_2=np.array([overlaped_2[-1][:2],overlaped_2[-2][:2]])
    
    overlaped_1=overlaped_1[1]-overlaped_1[0]
    overlaped_2=overlaped_2[1]-overlaped_2[0]
    
    magnitude_1=np.linalg.norm(overlaped_1)
    magnitude_2=np.linalg.norm(overlaped_2)

    try:
      theta=np.arccos(np.dot(overlaped_1,overlaped_2)/(magnitude_1*magnitude_2))
    except ZeroDivisionError:
      theta=0

    #check for threshold of angle
    if theta*(180/np.pi) > 0 & theta*(180/np.pi)<180:
      return True
    else:
      return False


def acc(trajectory,overlap,start,end,fun,fps,frameHeight):
  interval= 5
  tau=1/fps
  total_acc=[]

  for i in overlap:
    total_info=trajectory[i][start:end]
    speed=[]
    for i in range(len(total_info)-1):
      c1=np.asarray((total_info[i][0],total_info[i][1]))
      c2=np.asarray((total_info[i+1][0],total_info[i+1][1]))
      h1=total_info[i][2]
      h2=total_info[i+1][2]
      h_speed = (h1+h2)/2
      Gross_Speed=np.linalg.norm((c2-c1))/(tau*interval)
      scaled_speed=(((frameHeight-h_speed)/frameHeight)+1)*Gross_Speed
      speed.append(scaled_speed)
    acc=[]
    for j in range(len(speed)-1):
      A=(speed[j+1]-speed[j])/(tau*2*interval)
      acc.append(A)  
    total_acc.append(fun(acc))
  return total_acc


def max_mean_acc(trajectory,overlap,framenum,fps,height):
  avg_acc=acc (trajectory,overlap,framenum-3,framenum,np.mean,fps,height)
  max_acc=acc (trajectory,overlap,framenum,framenum+3,np.max,fps,height)
  if avg_acc[0]- max_acc[0] > 0.2  or  avg_acc[1]- max_acc[1] > 0.5 :
    return True
  else:
    return False




## To calculate the angle of each overlaped object
def direction_anomali(trajectories,overlaped):
  overlaped_1=np.array(trajectories[overlaped[0]])
  overlaped_2=np.array(trajectories[overlaped[1]])
  try:
    xobj1=(overlaped_1[-1][1]-overlaped_1[-2][1])/(overlaped_1[-1][0]-overlaped_1[-2][0])
  except ZeroDivisionError:
    xobj1=0
  try:
    xobj2=(overlaped_2[-1][1]-overlaped_2[-2][1])/(overlaped_2[-1][0]-overlaped_2[-2][0])
  except ZeroDivisionError:
    xobj2=0

  theta1=np.arctan(xobj1)*(180/np.pi)
  theta2=np.arctan(xobj2)*(180/np.pi)
  return [theta1,theta2]