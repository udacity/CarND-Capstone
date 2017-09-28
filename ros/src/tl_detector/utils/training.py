import glob
import numpy as np
import cv2
from sklearn.model_selection import train_test_split
import tensorflow as tf

#need to set the data path
nothing = glob.glob("./traffic_lights/no-light/*.jpg")
redlights = glob.glob("./traffic_lights/red/*.jpg")
greenlights = glob.glob("./traffic_lights/green/*.jpg")
yellowlights = glob.glob("./traffic_lights/yellow/*.jpg")


#Y = np.concatenate([np.ones(len(redlights)), np.zeros(len(nothing))-1])
Y = np.concatenate([np.ones(len(greenlights)), np.zeros(len(nothing))-1])
Y = np.concatenate([np.array([[1,0,0,0]]*len(nothing)), 
                    np.array([[0,1,0,0]]*len(redlights)),
                    np.array([[0,0,1,0]]*len(greenlights)),
                    np.array([[0,0,0,1]]*len(yellowlights))
                   ])

# Read X Vector
X = []
for name in nothing: 
    img = cv2.imread(name)
    (b,g,r) = cv2.split(img)
    X.append(cv2.merge([r, g, b]))   
    #X.append(cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB))
    
for name in redlights: 
    img = cv2.imread(name)
    (b,g,r) = cv2.split(img)
    X.append(cv2.merge([r, g, b]))   
    #X.append(cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB))
    
for name in greenlights:  
    img = cv2.imread(name)
    (b,g,r) = cv2.split(img)
    X.append(cv2.merge([r, g, b]))   
    #X.append(cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB))
    
for name in yellowlights:    
    img = cv2.imread(name)
    (b,g,r) = cv2.split(img)
    X.append(cv2.merge([r, g, b]))   
    #X.append(cv2.cvtColor(cv2.imread(name), cv2.COLOR_BGR2RGB))


X = np.array(X)


X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.10, random_state=42)


X_train = X_train.astype('float32')
X_test = X_test.astype('float32')
print('X_train shape:', X_train.shape)
print(X_train.shape[0], 'train samples')
print(X_test.shape[0], 'test samples')
input_shape =  (3,64,64)

def model(data, train):
    norm = data/127.5 - 1
    c1 = tf.layers.conv2d(norm, 16, 3, strides=(1, 1), padding='SAME',
                            kernel_initializer=tf.truncated_normal_initializer(stddev=0.001), activation=tf.nn.relu)
    c2 = tf.layers.conv2d(c1, 16, 3, strides=(1, 1), padding='SAME',
                            kernel_initializer=tf.truncated_normal_initializer(stddev=0.001), activation=tf.nn.relu)
    p = tf.layers.max_pooling2d(c2, 8, strides=(8,8), padding='SAME')
    
    drop1 = tf.layers.dropout(p,rate=0.25,training=train)
    
    c3 = tf.layers.conv2d(drop1, 128, 8, strides=(1, 1), padding='VALID',
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.001), activation=tf.nn.relu)
    
    drop2 = tf.layers.dropout(c3,rate=0.50,training=train)
    c4 = tf.layers.conv2d(drop2, 4, 1, strides=(1, 1), padding='VALID',
        kernel_initializer=tf.truncated_normal_initializer(stddev=0.001))
    sm = tf.nn.softmax(c4, name="output")
    return c4,sm
    
def loss(data, correct_label):    
    logits = tf.reshape(data,(-1,4),name='logits')
    labels = tf.reshape(correct_label,(-1,4),name='lables')
    loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=labels))
    train_op = tf.train.AdamOptimizer(0.001).minimize(loss)
    
    correct_prediction = tf.equal(tf.argmax(logits,1), tf.argmax(labels,1))
    tf_accuracy = tf.reduce_mean(tf.cast(correct_prediction, "float"))
    return loss,train_op,tf_accuracy



#need to set the test data path
#testdata = (cv2.cvtColor(cv2.imread("/home/kosuke/sf/data/testimagesreal/just00151.jpg"), cv2.COLOR_BGR2RGB))

batches = (int(np.ceil(X_train.shape[0]/64.)))


with tf.Session() as sess:
    
    
    tf_data = tf.placeholder(tf.float32, shape=(None, None, None, 3), name="image_input")
    tf_labels = tf.placeholder(tf.float32, shape=(None),name="labels_input")
    tf_train = tf.placeholder(tf.bool,name="train_input")
    
    modelout,sm = model(tf_data, tf_train)
    l,train_op, tf_accuracy = loss(modelout, tf_labels)

    sess.run(tf.global_variables_initializer())
    
    for epoch_i in range(200):
        for batch_i in range(batches):
            train_loss, _, accuracy = sess.run([l, train_op, tf_accuracy],
                        feed_dict={tf_data: X_train[batch_i*64:batch_i*64+64], 
                                   tf_labels: Y_train[batch_i*64:batch_i*64+64], 
                                   tf_train: True})
            # Display the loss after every tenth batch
            

        # Display the loss after the epoch
       
        print('Epoch {:>3} Batch {:>2}   Training loss = {:.3f} Acc:{:.3f}'.format(epoch_i+1, batch_i+1, train_loss, accuracy))
        
        train_loss, accuracy = sess.run([l, tf_accuracy],feed_dict={tf_data: X_test, tf_labels: Y_test, 
                                   tf_train: False})
        print('Test: Epoch {:>3} Batch {:>2}   Training loss = {:.3f} Acc:{:.3f}'.format(epoch_i+1, batch_i+1, train_loss, accuracy))
        
        
    
    saver = tf.train.Saver()
    saver.save(sess, "tl_cnn_weight",global_step=1000)
