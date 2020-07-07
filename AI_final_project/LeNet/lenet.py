from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import numpy as np
import time
from numpy.lib.stride_tricks import as_strided


# Example Sigmoid
# 这个类中包含了 forward 和backward函数
class Sigmoid():
    def __init__(self):
        pass

    def forward(self, x):
        return 1.0 / (1.0 + np.exp(-x))

    def backward(self, z):
        return self.forward(z) * (1 - self.forward(z))





## 在原 LeNet-5上进行少许修改后的 网路结构
"""
conv1: in_channels: 1, out_channel:6, kernel_size=(5x5), pad=0, stride=1, activation: relu
avgpool1: in_channels: 6, out_channels:6, kernel_size = (2x2), stride=2
conv2: in_channels: 6, out_channel:16, kernel_size=(5x5), pad=0, stride=1, activation: relu
avgpool2: in_channels: 16, out_channels:16, kernel_size = (2x2), stride=2
flatten
fc1: in_channel: 256, out_channels: 128, activation: relu
fc2: in_channel: 128, out_channels: 64, activation: relu
fc3: in_channel: 64, out_channels: 10, activation: relu
softmax:

tensor: (1x28x28)   --conv1    -->  (6x24x24)
tensor: (6x24x24)   --avgpool1 -->  (6x12x12)
tensor: (6x12x12)   --conv2    -->  (16x8x8)
tensor: (16x8x8)    --avgpool2 -->  (16x4x4)
tensor: (16x4x4)    --flatten  -->  (256)
tensor: (256)       --fc1      -->  (128)
tensor: (128)       --fc2      -->  (64)
tensor: (64)        --fc3      -->  (10)
tensor: (10)        --softmax  -->  (10)
"""
class Flatten(object):
    def __init__(self):
        pass
    
    def forward(self,X):
        self.shape_in = X.shape
        return X.reshape(X.shape[0],-1)
    
    def backward(self,delta):
        return delta.reshape(self.shape_in)


class Avgpool(object):
    def __init__(self,in_channel,out_channel,kernel_size,stride):
        self.Ci = in_channel
        self.Co = out_channel
        self.stride = stride
        

    def forward(self,X,Ho,Wo):
        B,_,Hi,Wi = X.shape
        div = self.stride**2
        self.div = div
        shape = (B,self.Co,Ho,Wo,self.stride,self.stride)
        strides = (*X.strides[:-2],X.strides[-2]*self.stride,X.strides[-1]*self.stride,*X.strides[-2:])
        return np.mean(as_strided(X,shape = shape,strides = strides),axis = (-2,-1))
    
    def backward(self,delta):
        return np.repeat(np.repeat(delta,self.stride,axis = -1),self.stride,axis = -2)/self.div

class Fc(object):
    def __init__(self,size_in,size_out):
        self.size_in = size_in
        self.size_out = size_out
        self.Weight = np.zeros((size_in,size_out))
        
    def init_weight(self):
        self.timestep = 0
        self.m1 = 0
        self.m2 = 0
        self.v1 = 0
        self.v2 = 0
        self.b1 = 0.9
        self.b2 = 0.999
        self.e = 1e-8
        np.random.seed()
        r = np.sqrt(6/(self.size_in + self.size_out))
        self.Weight = np.random.uniform(-r,r,(self.size_in,self.size_out))
        self.bias = np.random.uniform(-r,r,self.size_out)
        
    
    def forward(self,X):
        Data_in = X.reshape(-1,self.size_in)
        self.Data_in = Data_in
        return np.dot(X,self.Weight)+self.bias

    def backward(self,delta,lr):
        dx = np.tensordot(delta,self.Weight,axes = [(1,),(1,)])
        dw = np.tensordot(self.Data_in,delta,axes = [(0,),(0,)]) 
        db = np.sum(delta,axis = 0) 
        self.timestep += 1
        self.m1 = self.b1*self.m1 + (1-self.b1)*dw
        self.m2 = self.b1*self.m2 + (1-self.b1)*db        
        self.v1 = self.b2*self.v1 + (1-self.b2)*dw**2
        self.v2 = self.b2*self.v2 + (1-self.b2)*db**2
        mtt1 = self.m1/(1-self.b1**(self.timestep))
        vtt1 = self.v1/(1-self.b2**(self.timestep))   
        mtt2 = self.m2/(1-self.b1**(self.timestep))
        vtt2 = self.v2/(1-self.b2**(self.timestep))   
        self.Weight -= lr * mtt1 / (self.e + np.sqrt(vtt1))
        self.bias -= lr * mtt2 / (self.e + np.sqrt(vtt2))
        return dx
    
class Relu(object):
    def __init__(self):
        pass
    
    def forward(self,X):
        self.Data_in = X
        Out = X.copy()
        Out[Out<0] *= 0.01 
        return Out
    
    def backward(self,delta):
        dx = np.ones_like(self.Data_in)
        dx[self.Data_in <= 0] = 0.01
        return dx * delta
        
        
class Conv(object):
    
    def __init__(self,in_channel,out_channel,kernel_size,pad,stride):
        "kernel_size:Co x Ci x kh x kw"
        self.kernel = np.zeros(kernel_size)
        self.Ci = in_channel
        self.Co = out_channel
        self.pad = pad
        self.stride = stride
        
    def init_weight(self,nin,nout):
        self.timestep = 0
        self.m = 0
        self.v = 0
        self.b1 = 0.9
        self.b2 = 0.999
        self.e = 1e-8
        np.random.seed()
        kh,kw = self.kernel.shape
        r = np.sqrt(6/(nin+nout))
        self.kernel = np.random.uniform(-r,r,(self.Co,self.Ci,kh,kw))
        

    
    def forward(self,X):
        "X:B x Ci x Hi x Wi"
        "Out:B x Co x Ho x Wo"
        Co,Ci,kh,kw = self.kernel.shape
        B,_,h,w = X.shape
        Ho,Wo = ((h+2*self.pad-kh)//self.stride+1,(w+2*self.pad-kw)//self.stride+1)
        shape = (B,Ci,Ho,Wo,kh,kw)
        X = np.pad(X,((0,),(0,),(self.pad,),(self.pad,)),'constant')
        self.Data_in = X
        strides = (*X.strides[:-2],X.strides[-2]*self.stride,X.strides[-1]*self.stride,*X.strides[-2:])
        A = as_strided(X,shape = shape,strides = strides,writeable = False)
        return np.tensordot(A,self.kernel,axes = [(1,4,5),(1,2,3)]).transpose((0,3,1,2))
    
    
    def backward(self,delta,lr):
        Co,Ci,kh,kw = self.kernel.shape
        _,_,Ho,Wo = delta.shape
        B,_,Hi,Wi = self.Data_in.shape
        delta_pad = np.pad(delta,((0,),(0,),(kh-1,),(kw-1,)),'constant')
        kernel_rot = np.rot90(self.kernel,2,(-2,-1))
        X_s = self.Data_in.strides
        shape1 = (B,Ci,kh,kw,Ho,Wo)
        strides1 = (*X_s[:-2],*X_s[-2:],*X_s[-2:])
        A1 = as_strided(self.Data_in,shape = shape1,strides = strides1)
        dw = np.tensordot(delta,A1,axes = [(0,2,3),(0,4,5)])

        shape2 = (B,Co,Hi,Wi,kh,kw)
        strides2 = (*delta_pad.strides[:-2],*delta_pad.strides[-2:],*delta_pad.strides[-2:])
        A2 = as_strided(delta_pad,shape = shape2,strides = strides2)
        dx = np.tensordot(A2,kernel_rot,axes = [(1,4,5),(0,2,3)]).transpose((0,3,1,2))
        
        self.timestep += 1
        self.m = self.b1*self.m + (1-self.b1)*dw
        self.v = self.b2*self.v + (1-self.b2)*dw**2
        mtt = self.m/(1-self.b1**(self.timestep))
        vtt = self.v/(1-self.b2**(self.timestep))
        self.kernel -= lr * mtt / (self.e + np.sqrt(vtt))
        return dx   
            
class SoftmaxLoss(object):
    def __init__(self,size):
        "size：类别数"
        self.size = size
        
    def forward(self,X):
        B,size = X.shape
        self.batch = B
        Out = np.zeros_like(X)
        for b in range(B):
            Out[b] = np.exp(X[b])/np.sum(np.exp(X[b]))
        self.result = Out
        return Out
    
    def backward(self,ind):
        Ind = ind.reshape(self.batch,-1)
        dx = self.result.copy()
        for b in range(self.batch):
            dx[b,Ind[b]] -= 1
        return dx
    

class LeNet(object):
    def __init__(self):
        '''
        初始化网路，在这里你需要，声明各Conv类， AvgPool类，Relu类， FC类对象，SoftMax类对象
        并给 Conv 类 与 FC 类对象赋予随机初始值
        注意： 不要求做 BatchNormlize 和 DropOut, 但是有兴趣的可以尝试
        '''
        self.conv1 = Conv(1,6,(5,5),0,1)
        self.relu1 = Relu()
        self.avgpool1 = Avgpool(6,6,(2,2),2)
        self.conv2 = Conv(6,16,(5,5),0,1)
        self.relu2 = Relu()
        self.avgpool2 = Avgpool(16,16,(2,2),2)
        self.flatten = Flatten()
        self.fc1 = Fc(256,128)
        self.relu3 = Relu()
        self.fc2 = Fc(128,64)
        self.relu4 = Relu()
        self.fc3 = Fc(64,10)
        self.relu5 = Relu()
        self.softmax = SoftmaxLoss(10)
        self.init_weight()
        print("initialize")

    def init_weight(self):
        self.conv1.init_weight(nin = 1*24*24*5*5, nout = 6*24*24)
        self.conv2.init_weight(nin = 6*8*8*5*5, nout = 16*8*8)
        self.fc1.init_weight()
        self.fc2.init_weight()
        self.fc3.init_weight()
        

    def forward(self, x):
        """前向传播
        x是训练样本， shape是 B,C,H,W
        这里的C是单通道 c=1 因为 MNIST中都是灰度图像
        返回的是最后一层 softmax后的结果
        也就是 以 One-Hot 表示的类别概率

        Arguments:
            x {np.array} --shape为 B，C，H，W
        """
        x1 = self.relu1.forward(self.conv1.forward(x))
        x2 = self.avgpool1.forward(x1,12,12)
        x3 = self.relu2.forward(self.conv2.forward(x2))
        x4 = self.avgpool2.forward(x3,4,4)
        x4_flat = self.flatten.forward(x4)
        x5 = self.relu3.forward(self.fc1.forward(x4_flat))
        x6 = self.relu4.forward(self.fc2.forward(x5))
        x7 = self.relu5.forward(self.fc3.forward(x6))
        result = self.softmax.forward(x7)
        return result

    def backward(self, index_true, lr=1.0e-3):
        """根据error，计算梯度，并更新model中的权值
        Arguments:
            error {np array} -- 即计算得到的loss结果
            lr {float} -- 学习率，可以在代码中设置衰减方式
        """
        delta1 = self.softmax.backward(index_true)
        delta2 = self.fc3.backward(self.relu5.backward(delta1),lr = lr)
        delta3 = self.fc2.backward(self.relu4.backward(delta2),lr = lr)
        delta4 = self.fc1.backward(self.relu3.backward(delta3),lr = lr)
        delta4_flat = self.flatten.backward(delta4)
        delta5 = self.avgpool2.backward(delta4_flat)
        delta6 = self.conv2.backward(self.relu2.backward(delta5),lr = lr)
        delta7 = self.avgpool1.backward(delta6)
        self.conv1.backward(self.relu1.backward(delta7),lr = lr)
        

    def evaluate(self, x, labels):
        """
        x是测试样本， shape 是BCHW
        labels是测试集中的标注， 为one-hot的向量
        返回的是分类正确的百分比

        在这个函数中，建议直接调用一次forward得到pred_labels,
        再与 labels 做判断

        Arguments:
            x {np array} -- BCWH
            labels {np array} -- B x 10
        """
        B,H,W = x.shape
        x = x.reshape(B,1,H,W)
        pred = self.forward(x)
        label = np.argmax(labels,axis = 1)
        result = np.argmax(pred,axis = 1)
        acc = np.zeros_like(result)
        acc[result == label] = 1
        return np.sum(acc)/np.size(acc)

    def data_augmentation(self, images):
        '''
        数据增强，可选操作，非强制，但是需要合理
        一些常用的数据增强选项： ramdom scale， translate， color(grayscale) jittering， rotation, gaussian noise,
        这一块儿允许使用 opencv 库或者 PIL image库
        比如把6旋转90度变成了9，但是仍然标签为6 就不合理了
        '''
        return images

    def compute_loss(self,result,labels):
        loss = 0
        for b in range(result.shape[0]):
            index = np.argmax(labels[b])
            loss -= np.log(result[b,index])
        return loss
    
    def fit(
        self,
        train_image,
        train_label,
        test_image = None,
        test_label = None,
        epoches = 10,
        batch_size = 16,
        lr = 1.0e-3,
    ):
        sum_time = 0
        accuracies = []
        for epoch in range(epoches):
            ## 可选操作，数据增强
            train_image = self.data_augmentation(train_image)
            ## 随机打乱 train_image 的顺序， 但是注意train_image 和 test_label 仍需对应
            '''
            # 1. 一次forward，bachword肯定不能是所有的图像一起,
            因此需要根据 batch_size 将 train_image, 和 train_label 分成: [ batch0 | batch1 | ... | batch_last]
            '''
            batch_images = train_image.reshape(-1,batch_size,1,28,28) # 请实现 step #1
            batch_labels = train_label.reshape(-1,batch_size,10) # 请实现 step #1
            last = time.time() #计时开始
            for imgs, labels in zip(batch_images, batch_labels):
                '''
                这里我只是给了一个范例， 大家在实现上可以不一定要按照这个严格的 2,3,4步骤
                我在验证大家的模型时， 只会在main中调用 fit函数 和 evaluate 函数。
                2. 做一次forward，得到pred结果  eg. pred = self.forward(imgs)
                3. pred 和 labels做一次 loss eg. error = self.compute_loss(pred, labels)
                4. 做一次backward， 更新网络权值  eg. self.backward(error, lr=1e-3)
                '''
                pred = self.forward(imgs)
                index_true = np.argmax(labels,axis = 1)
                self.backward(index_true,lr)
            duration = time.time() - last
            sum_time += duration

            if epoch % 5 == 0:
                accuracy = self.evaluate(test_image, test_label)
                print("epoch{} accuracy{}".format(epoch, accuracy))
                accuracies.append(accuracy)

        avg_time = sum_time / epoches
        return avg_time, accuracies


