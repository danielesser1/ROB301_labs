import numpy as np

# [ 0,     1,      2,     3,     4,     5  ]
# [Red, Orange, Yellow, Green, Blue, Purple]


class BayesLoc:

   def __init__(self, P0, colourCodes,colourMap,transProbBack,transProbForward):
      self.probability = P0
      self.colourCodes = colourCodes
      self.colourMap = colourMap
      self.transProbBack = transProbBack
      self.transProbForward = transProbForward
      self.numStates = len(P0)
      self.statePrediction = np.zeros(np.shape(P0))

   def getSensorReading(self,CurColour):
      dist = np.linalg.norm(self.colourCodes-CurColour,axis = 1)
      dist += np.ones(np.shape(dist))*.01
      prob = dist**-1
      prob = prob/np.sum(prob)
      return prob
   
   def statePredict(self,forward):
      # if forward == True, uk = +1, else uk = -1. 
      #self.probability = [.1, .2, .5, .2]
      newProb = np.zeros(np.shape(self.probability))
      for i in range(self.numStates):
          if forward:
              for j in range(0,len(self.transProbForward)):
                  if i+j < self.numStates:
                      newProb[i+j] += self.transProbForward[j]*self.probability[i]
          else:
              for j in range(0,len(self.transProbBack)):
                  if i-j < self.numStates:
                      newProb[i-j] += self.transProbBack[j]*self.probability[i]
      self.statePrediction = newProb/np.sum(newProb) #normalize due to edge effects

   def stateUpdate(self,RGBFromCamera):
      # if forward == True, uk = +1, else uk = -1. 
      colourProbs = self.getSensorReading(RGBFromCamera)
      #self.probability = [.1, .2, .5, .2]
      newProb = np.zeros(np.shape(self.probability))
      for i in range(self.numStates):
          newProb[i] = self.statePrediction[i]*colourProbs[self.colourMap[i]]
      self.probability = newProb/np.sum(newProb)
      
