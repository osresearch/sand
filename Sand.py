# "LED sand" pixel simulator
# Based on Adafruit_PixelDust.cpp, written by Phil "PaintYourDragon" Burgess for Adafruit Industries.
# BSD license, all text here must be included in any redistribution.
#
# This handles the "physics engine" part of a sand/rain simulation.
# The term "physics" is used loosely here...it's a relatively crude
# algorithm that's appealing to the eye but takes many shortcuts with
# collision detection, etc.
import random
from math import sqrt

def rand(low,high):
        return ((random.getrandbits(20) * (high - low)) >> 20) + low

# 1-axis elastic bounce, 0 to 256. higher is "bouncier"
elasticity = 180
def bounce(n):
	return int((-n * elasticity) / 256)

class Bitmap:
	def __init__(self, w, h):
		self.w = w
		self.h = h
		self.bitmap = bytearray(int(w*h))
	def get(self, x, y):
		#y_mask = y & 7
		#y_stride = y >> 3
		try:
			#v = self.bitmap[x + y_stride * self.w]
			#return (v & y_mask) != 0
			v = self.bitmap[x + y * self.w]
			return v != 0
		except:
			print("OUT OF RANGE %d x %d" % (x, y))
			raise
	def set(self, x, y):
		#y_mask = y & 7
		#y_stride = y >> 3
		#self.bitmap[x + y_stride * self.w] |= y_mask
		self.bitmap[x + y * self.w] = 1
	def clear(self, x, y):
		#y_mask = y & 7
		#y_stride = y >> 3
		#self.bitmap[x + y_stride * self.w] &= ~y_mask
		self.bitmap[x + y * self.w] = 0

class Grain:
	def __init__(self, w, h, bitmap):
		self.vx = 0
		self.vy = 0
		self.w = (w << 8) - 1
		self.h = (h << 8) - 1
		while True:
			# find free random location
			self.px = rand(0,w)
			self.py = rand(0,h)
			if bitmap.get(self.px, self.py):
				continue

			self.x = self.px << 8
			self.y = self.py << 8
			bitmap.set(self.px, self.py)
			break

	def update_vel(self, ax, ay, az):
		self.vx += ax + rand(0,az)
		self.vy += ay + rand(0,az)
		v2 = self.vx * self.vx + self.vy * self.vy

		#// Terminal velocity (in any direction) is 256 units -- equal to
		#// 1 pixel -- which keeps moving grains from passing through each other
		#// and other such mayhem.  Though it takes some extra math, velocity is
		#// clipped as a 2D vector (not separately-limited X & Y) so that
		#// diagonal movement isn't faster than horizontal/vertical.
		if v2 > 65535:
			v = sqrt(v2)
			self.vx = int(256 * self.vx / v)
			self.vy = int(256 * self.vy / v)


	def update_pos(self, bitmap):
		#// ...then update position of each grain, one at a time, checking for
		#// collisions and having them react.  This really seems like it shouldn't
		#// work, as only one grain is considered at a time while the rest are
		#// regarded as stationary.  Yet this naive algorithm, taking many not-
		#// technically-quite-correct steps, and repeated quickly enough,
		#// visually integrates into something that somewhat resembles physics.
		#// (I'd initially tried implementing this as a bunch of concurrent and
		#// "realistic" elastic collisions among circular grains, but the
		#// calculations and volume of code quickly got out of hand for both
		#// the tiny 8-bit AVR microcontroller and my tiny dinosaur brain.)
		new_x = self.x + self.vx # // New position in grain space
		new_y = self.y + self.vy

		# Check for the bounding box
		if new_x < 0:
			new_x = 0 # keep it inside the box
			self.vx = bounce(self.vx) # and bounce off wall
		elif new_x > self.w:
			new_x = self.w # keep it inside the box
			self.vx = bounce(self.vx) # and bounce off wall

		if new_y < 0:
			new_y = 0 # keep it inside,
			self.vy = bounce(self.vy) # and bounce off wall
		elif new_y > self.h:
			new_y = self.h # keep it inside,
			self.vy = bounce(self.vy) # and bounce off wall

		# moving to a new pixel?
		new_px = new_x >> 8
		new_py = new_y >> 8

		if new_px == self.px and new_py == self.py:
			# still in the same pixel, bitmap is unchanged
			self.x = new_x
			self.y = new_y
			return

		if not bitmap.get(new_px, new_py):
			# motion to a clear new pixel,
			# nothing to change here
			pass
		elif new_py == self.py:
			# collision only in the x direction (same py)
			new_x = self.x # cancel X motion, stay in this pixel
			new_px = self.px
			self.vx = bounce(self.vx) # and bounce X velocity
		elif new_px == self.px:
			# collision only in the y direction (same px)
			new_y = self.y # cancel Y motion, stay in this pixel
			new_py = self.py
			self.vy = bounce(self.vy) # and bounce Y velocity
		elif abs(self.vx) >= abs(self.vy):
			# diagonal collision, X axis velocity is faster so try Y first
			if not bitmap.get(new_px, self.py):
				# That pixel's free!  Take it!  But...
				new_y = self.y # Cancel Y motion
				new_py = self.py
				self.vy = bounce(self.vy) # and bounce Y velocity
			elif not bitmap.get(self.px, new_py):
				new_x = self.x # Cancel X motion
				new_px = self.px
				self.vx = bounce(self.vx) # and bounce X velocity
			else:
				# Both spots are occupied, cancel motion and bounce velocity
				self.vx = bounce(self.vx)
				self.vy = bounce(self.vy)
				return
		else:
			# diagonal collision with Y axis faster, start there
			if not bitmap.get(self.px, new_py):
				new_x = self.x # Cancel X motion
				new_px = self.px
				self.vx = bounce(self.vx) # and bounce X velocity
			elif not bitmap.get(new_px, self.py):
				# That pixel's free!  Take it!  But...
				new_y = self.y # Cancel Y motion
				new_py = self.py
				self.vy = bounce(self.vy) # and bounce Y velocity
			else:
				# Both spots are occupied, cancel motion and bounce velocity
				self.vx = bounce(self.vx)
				self.vy = bounce(self.vy)
				return

		# move to the new pixel
		bitmap.clear(self.px, self.py)
		bitmap.set(new_px, new_py)
		self.px = new_px
		self.py = new_py
		self.x = new_x
		self.y = new_y

class Sand:
	def __init__(self, w, h, n, scale=1):
		self.w = w
		self.h = h
		self.n = n
		self.scale = scale
		self.randomize()

	def randomize(self):
		self.bitmap = Bitmap(self.w,self.h)
		self.grains = [Grain(self.w,self.h,self.bitmap) for x in range(self.n)]

	# Calculate one frame of particle interactions
	def update(self,ax,ay,az):
		ax = int(ax * self.scale) >> 8 #  // Scale down raw accelerometer
		ay = int(ay * self.scale) >> 8 #  // inputs to manageable range.
		az = abs(int(az * self.scale) >> 12) # // Z is further scaled down 1:8

		# // A tiny bit of random motion is applied to each grain, so that tall
		# // stacks of pixels tend to topple (else the whole stack slides across
		# // the display).  This is a function of the Z axis input, so it's more
		# // pronounced the more the display is tilted (else the grains shift
		# // around too much when the display is held level).

		# Clip & invert
		if az >= 4:
			az = 1
		else:
			az = 5 - az
		ax -= az # Subtract Z motion factor from X, Y,
		ay -= az # then...
		az2 = az * 2 + 1 # max random motion to add back in

		# Apply 2D accel vector to grain velocities...
		for grain in self.grains:
			grain.update_vel(ax, ay, az2)

		# Update the position, checking for collisions in the bitmap
		for grain in self.grains:
			grain.update_pos(self.bitmap)

if __name__ == "__main__":
	s = Sand(128, 32, 256)
	
	while True:
		s.update(1810, 00, 100)
		print('%c[H' % (27))

		for y in range(0,s.h):
			for x in range(0,s.w):
				if s.bitmap.get(x,y):
					print("0", end='')
				else:
					print(" ", end='')
			print("")
