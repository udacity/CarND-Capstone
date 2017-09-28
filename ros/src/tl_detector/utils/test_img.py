import tensorflow as tf
import cv2
import numpy as np
import glob
outcounter = 0


with tf.Session() as sess:    
    saver = tf.train.import_meta_graph('tl_cnn_weight-1000.meta')
    saver.restore(sess,tf.train.latest_checkpoint('./'))
    graph = tf.get_default_graph()
    
    

    tf_data = graph.get_tensor_by_name("image_input:0")
    tf_train = graph.get_tensor_by_name("train_input:0")
    tf_label = graph.get_tensor_by_name("labels_input:0")    
    sm = graph.get_tensor_by_name("output:0")
    
    #files = glob.glob("./SDC/object-detection-crowdai/*.jpg")
    #files = glob.glob("./SDC/Udacity-loop/*.jpg")
    files = glob.glob("./SDC/sim_imgs/*.jpg")
    fileindex = 0
    print(files[fileindex])

    img = cv2.imread(files[fileindex])
    testdata = (cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    dsp = img.copy()
    cv2.imshow('frame', dsp)

    while True:

	k = cv2.waitKey(1)
	
	if k != 255:
		#print(k)
		pass
	if k == ord("q"):
		break
	
	if k == ord("n"):
		fileindex= fileindex + 1
		image = cv2.imread(files[fileindex])
		testdata = (cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
		#dsp = img.copy()
		#cv2.imshow('frame', dsp)
		img = sess.run([sm], feed_dict={tf_data: testdata.reshape([1,600, 800, 3]), tf_train: False})
		threshold = 0.9999
		img = np.array(img)
		r = np.sum(img[0,0,:,:,1]>threshold)
		g = np.sum(img[0,0,:,:,2]>threshold)
		y = np.sum(img[0,0,:,:,3]>threshold)

		if((r+g+y)>20) :
			if(r>=g and r>=y):
				print(" It is red light. ")
			elif(g>y and g>15):
				print(" It is green light. ")
			elif(y>10):
				print(" It is yellow light. ")

		else:
			print (" I don't know. ")

		print("Nothing:",np.sum(img[0,0,:,:,0]>threshold))
		print("Red:",np.sum(img[0,0,:,:,1]>threshold))
		print("Green:",np.sum(img[0,0,:,:,2]>threshold))
		print("Yellow:",np.sum(img[0,0,:,:,3]>threshold))

    

    		def augment(raw, boolean, color):
        		xx, yy = np.meshgrid(np.arange(img.shape[3]),np.arange(img.shape[2]))

			x = (xx[boolean])
			y = (yy[boolean])

			for i,j in zip(x,y):
				cv2.rectangle(testdata, (i*8,j*8), (i*8+64,j*8+64), color, 2)
            
		augment(testdata,img[0,0,:,:,1]>threshold,(255,0,0))
		augment(testdata,img[0,0,:,:,2]>threshold,(0,255,0))
		augment(testdata,img[0,0,:,:,3]>threshold,(255,255,0))

		cv2.imshow('frame', cv2.cvtColor(testdata, cv2.COLOR_RGB2BGR))
		

