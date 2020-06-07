
#Image Publisher
# def startCapturing(self):
#         rospy.loginfo("[%s] Start capturing." %(self.node_name))
#         while not self.is_shutdown and not rospy.is_shutdown():
#             gen =  self.grabAndPublish(self.stream,self.pub_img)
#             try:
#                 self.camera.capture_sequence(gen,'jpeg',use_video_port=True,splitter_port=0)
#             except StopIteration:
#                 pass
#             print "updating framerate"
#             self.camera.framerate = self.framerate
#             self.update_framerate=False

#         self.camera.close()
#         rospy.loginfo("[%s] Capture Ended." %(self.node_name))

# def grabAndPublish(self,stream,publisher):
#     while not self.update_framerate and not self.is_shutdown and not rospy.is_shutdown(): 
#         yield stream
#         # Construct image_msg
#         # Grab image from stream
#         stamp = rospy.Time.now()
#         stream.seek(0)
#         stream_data = stream.getvalue()
#         # Generate compressed image
#         image_msg = CompressedImage()
#         image_msg.format = "jpeg"
#         image_msg.data = stream_data

#         image_msg.header.stamp = stamp
#         image_msg.header.frame_id = self.frame_id
#         publisher.publish(image_msg)
					
#         # Clear stream
#         stream.seek(0)
#         stream.truncate()
		
#         if not self.has_published:
#             rospy.loginfo("[%s] Published the first image." %(self.node_name))
#             self.has_published = True

#         rospy.sleep(rospy.Duration.from_sec(0.001))