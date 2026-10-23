import torch
import torch.nn.functional as F

def focal_R_loss(pred, target, beta=5, gamma=3):
    f1score = F.l1_loss(pred,target,reduction='none')
    weight = torch.abs(torch.mul(f1score,beta))
    weight = torch.sigmoid(weight)
    weight = torch.pow(weight,gamma)
    loss = torch.mul(weight, f1score)
    return loss.mean()

# Following the paper "Keyframe-Focused Visual Imitation Learning", the loss is weighted using softmax or step function.
# Because of the reported results in that paper, we use softmax, the step fuction needs a threshold.
# Threshold is determined using the percentile, outside this function.

def step_weights(copycat_loss, threshold, step=5):
    weights = torch.where(torch.gt(copycat_loss,threshold),step,1)
    return weights

#The paper reports better results with sofmax of the minibatch.
def softmax_weights(copycat_loss, temperature=0.2):
    exponential=torch.exp(copycat_loss)**temperature
    sum=torch.sum(exponential)
    weights=exponential/sum
    return weights

def RBC_loss(pred,target,weights):
    f1score = F.l1_loss(pred,target,reduction='none')
    loss = torch.mul(f1score, weights)
    return loss.mean()

def softmin_temperature(logits, temperature):
    logits = logits / temperature
    return F.softmin(logits, dim=-1)
    
