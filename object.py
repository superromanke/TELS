import time

class object:
	def __init__(self, id, category):
		self.id = id
		self.updated = True
		self.category = category
		self.tracks = []
	def getID(self):
		return self.id
	def getUpdated(self):
		return self.updated
	def getCategory(self):
		return self.category
	def getTracks(self):
		return self.tracks
	def setID(self, setid):
		self.id = setid
	def setUpdated(self,update):
		self.updated = update
	def add2tracks(self,track):
		if len(track) == 4:
			track.append(time.time())
			self.tracks.append(track)
		else:
			print('Fail to add to tracks')
