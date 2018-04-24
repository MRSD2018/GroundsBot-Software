#!/usr/bin/env python
import json
import io
import os

import config
import ee
import jinja2
import webapp2
import logging

from google.appengine.api import memcache
from google.appengine.ext import ndb

###############################################################################
#                             Web request handlers.                           #
###############################################################################

class MainHandler(webapp2.RequestHandler):
  """A servlet to handle requests to load the main grudsby web page."""
  def get(self, path=''):
    mapid = GetgrudsbyMapId()
    template_values = {
        'eeMapId': mapid['mapid'],
        'eeToken': mapid['token']
    }
    template = JINJA2_ENVIRONMENT.get_template('index.html')
    self.response.out.write(template.render(template_values))

class AdminHandler(webapp2.RequestHandler):
  """A servlet to handle requests to load the main grudsby web page."""
  def get(self, path=''):
    mapid = GetgrudsbyMapId()
    template_values = {
        'eeMapId': mapid['mapid'],
        'eeToken': mapid['token']
    }
    template = JINJA2_ENVIRONMENT.get_template('admin.html')
    self.response.out.write(template.render(template_values))

class GeofenceHandler(webapp2.RequestHandler):
  """A servlet to handle requests for details about a Geofence."""
  def get(self):
    polygon_id = self.request.get('polygon_id')
    loadedPath = getString(polygon_id + "_geofence")
    try:
      content = json.dumps(json.loads(loadedPath), sort_keys=True, indent=2)
    except ValueError, e:
      content = json.dumps({'error': 'Stored data not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)

class SaveGeofenceHandler(webapp2.RequestHandler):
  """A servlet to save details about a Region."""
  def get(self):
    rawJson = str(self.request.get('jsonData'))
    try:
      polygonData = json.loads(rawJson)
    except ValueError, e:
      False  
    if ('coordinates' in polygonData and 'regionID' in polygonData):
      content = json.dumps({'result': 'Successfully received region ' + polygonData['regionID']}) 
      setString(polygonData['regionID']+"_geofence", json.dumps(polygonData, ensure_ascii=False))
    else:
      content = json.dumps({'error': 'Request not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)


class RegionHandler(webapp2.RequestHandler):
  """A servlet to handle requests for details about a Region."""
  def get(self):
    polygon_id = self.request.get('polygon_id')
    loadedPath = getString(polygon_id + "_region")
    try:
      content = json.dumps(json.loads(loadedPath), sort_keys=True, indent=2)
    except ValueError, e:
      content = json.dumps({'error': 'Stored data not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)

class SaveRegionHandler(webapp2.RequestHandler):
  """A servlet to save details about a Region."""
  def get(self):
    rawJson = str(self.request.get('jsonData'))
    try:
      polygonData = json.loads(rawJson)
    except ValueError, e:
      False  
    if ('coordinates' in polygonData and 'regionID' in polygonData):
      content = json.dumps({'result': 'Successfully received region ' + polygonData['regionID']}) 
      setString(polygonData['regionID']+"_region", json.dumps(polygonData, ensure_ascii=False))
    else:
      content = json.dumps({'error': 'Request not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)

class MowingPlanHandler(webapp2.RequestHandler):
  """A servlet to handle requests for details about a Mowing Plan."""
  def get(self):
    polygon_id = self.request.get('polygon_id')
    loadedPath = getString(polygon_id + "_plan")
    try:
      content = json.dumps(json.loads(loadedPath), sort_keys=True, indent=2)
    except ValueError, e:
      content = json.dumps({'error': 'Stored data not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)

class SaveMowingPlanHandler(webapp2.RequestHandler):
  """A servlet to save details about a Mowing Region."""
  def get(self):
    rawJson = str(self.request.get('jsonData'))
    try:
      polygonData = json.loads(rawJson)
    except ValueError, e:
      False
    if ('coordinates' in polygonData and 'regionID' in polygonData):
      content = json.dumps({'result': 'Successfully received plan ' + polygonData['regionID']}) 
      setString(polygonData['regionID']+"_plan", json.dumps(polygonData, ensure_ascii=False))
    else:
      content = json.dumps({'error': 'Request not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)
  def post(self):
    dataJson = self.request.get('jsonData')
    try:
      polygonData = json.loads(dataJson)
    except ValueError, e:
      False
    if ('coordinates' in polygonData and 'regionID' in polygonData):
      content = json.dumps({'result': 'Successfully received plan ' + polygonData['regionID']}) 
      setString(polygonData['regionID']+"_plan", json.dumps(polygonData, ensure_ascii=False))
    else:
      content = json.dumps({'error': 'Request not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)

class ApprovalHandler(webapp2.RequestHandler):
  """A servlet to handle requests for details about an Approval."""
  def get(self):
    polygon_id = self.request.get('polygon_id')
    loadedPath = getString(polygon_id + "_approval")
    try:
      content = json.dumps(json.loads(loadedPath), sort_keys=True, indent=2)
    except ValueError, e:
      content = json.dumps({'error': 'Stored data not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)


class SetApprovalHandler(webapp2.RequestHandler):
  """A servlet to save an Approval."""
  def get(self):
    rawJson = str(self.request.get('jsonData'))
    try:
      polygonData = json.loads(rawJson)
    except ValueError, e:
      False
    if ('approval' in polygonData and 'regionID' in polygonData):
      content = json.dumps({'result': 'Successfully received approval ' + polygonData['regionID']}) 
      setString(polygonData['regionID']+"_approval", json.dumps(polygonData, ensure_ascii=False))
    else:
      content = json.dumps({'error': 'Request not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)

class MowerPosHandler(webapp2.RequestHandler):
  """A servlet to handle requests for details about an Approval."""
  def get(self):
    loadedPath = getString("mowerPos")
    try:
      content = json.dumps(json.loads(loadedPath), sort_keys=True, indent=2)
    except ValueError, e:
      content = json.dumps({'error': 'Stored data not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)


class SetMowerPosHandler(webapp2.RequestHandler):
  """A servlet to save an Approval."""
  def get(self):
    rawJson = str(self.request.get('jsonData'))
    try:
      mowerPos = json.loads(rawJson)
    except ValueError, e:
      False
    if ('lat' in mowerPos and 'lng' in mowerPos and 'rot' in mowerPos):
      content = json.dumps({'result': 'Successfully received moweor position.'}) 
      setString("mowerPos", json.dumps(mowerPos, ensure_ascii=False))
    else:
      content = json.dumps({'error': 'Request not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)


class ObstaclesHandler(webapp2.RequestHandler):
  """A servlet to handle requests for details about an Approval."""
  def get(self):
    loadedPath = getString("obstacles")
    try:
      content = json.dumps(json.loads(loadedPath), sort_keys=True, indent=2)
    except ValueError, e:
      content = json.dumps({'error': 'Stored data not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)


class ClearObstaclesHandler(webapp2.RequestHandler):
  """A servlet to handle requests for details about an Approval."""
  def get(self):
    setString('obstacles',"{\"obstacles\":[]}")
    content = json.dumps({'result':'cleared the obstacles'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)



class AddObstacleHandler(webapp2.RequestHandler):
  """A servlet to add an Obstacle."""
  def get(self):
    rawJson = str(self.request.get('jsonData'))
    try:
      obstaclePos = json.loads(rawJson)
    except ValueError, e:
      False
    if ('lat' in obstaclePos and 'lng' in obstaclePos):
      content = json.dumps({'result': 'Successfully received obstacle position.'}) 
      obstacles = getString("obstacles")
      try:
        obstaclesJSON = json.loads(obstacles)
        obstaclesJSON["obstacles"].append({"lat":obstaclePos['lat'],"lng":obstaclePos['lng']})
        setString("obstacles", json.dumps(obstaclesJSON)) 
      except ValueError, e:
        setString('obstacles',"{\"obstacles\":[{\"lat\":"+str(obstaclePos['lat'])+",\"lng\":"+str(obstaclePos['lng'])+"}]}")
        content = json.dumps({'error': 'Stored data not formatted correctly. Added new obstacles object'})
      
    else: 
      content = json.dumps({'error': ' Received data not formatted correctly'})
    self.response.headers['Content-Type'] = 'application/json'
    self.response.out.write(content)



# Define webapp2 routing from URL paths to web request handlers. See:
# http://webapp-improved.appspot.com/tutorials/quickstart.html
app = webapp2.WSGIApplication([
    ('/', MainHandler),
    ('/admin', AdminHandler),
    ('/geofence', GeofenceHandler),
    ('/saveGeofence', SaveGeofenceHandler),
    ('/region', RegionHandler),
    ('/saveRegion', SaveRegionHandler),
    ('/plan', MowingPlanHandler),
    ('/savePlan', SaveMowingPlanHandler),
    ('/approval', ApprovalHandler),
    ('/setApproval', SetApprovalHandler),
    ('/mowerPos', MowerPosHandler),
    ('/setMowerPos', SetMowerPosHandler),
    ('/obstacles', ObstaclesHandler),
    ('/clearObstacles', ClearObstaclesHandler),
    ('/addObstacle', AddObstacleHandler)
])


###############################################################################
#                                   Helpers.                                  #
###############################################################################


class StoredData(ndb.Model):
  """A main model for representing a data entry."""
  dataID = ndb.StringProperty(indexed=True) 
  storedData = ndb.StringProperty(indexed=False)

def getString(stringID):
  """Get a stored string based on ID"""
  data_query = StoredData.query(
    StoredData.dataID == stringID)
  data = data_query.fetch(1)
  if (len(data) == 0):
    return ""
  else:
    return data[0].storedData
    
def setString(stringID, newValue):
  """Store a string by ID and value"""
  data_query = StoredData.query(
    StoredData.dataID == stringID)
  data = data_query.fetch(1)
  if (len(data) == 0):
    newData = StoredData()
    newData.dataID = stringID
    newData.storedData = newValue
    newData.put()
  else:
    refreshedData = data[0]
    refreshedData.storedData = newValue
    refreshedData.put()
  

def GetgrudsbyMapId():
  """Returns the MapID for the night-time lights trend map."""
  collection = ee.ImageCollection(IMAGE_COLLECTION_ID).filterBounds(ee.Geometry.Rectangle(-79.944951, 40.434421, -79.812859, 40.550586)).filterDate('2015-01-01', '2018-12-31').mosaic()

  return collection.getMapId({
      'opacity': '0.01',
  })


###############################################################################
#                                   Constants.                                #
###############################################################################


# Memcache is used to avoid exceeding our EE quota. Entries in the cache expire
# 24 hours after they are added. See:
# https://cloud.google.com/appengine/docs/python/memcache/
MEMCACHE_EXPIRATION = 60 * 60 * 24

# The ImageCollection of the night-time lights dataset. See:
# https://earthengine.google.org/#detail/NOAA%2FDMSP-OLS%2FNIGHTTIME_LIGHTS
IMAGE_COLLECTION_ID = 'USDA/NAIP/DOQQ'

###############################################################################
#                               Initialization.                               #
###############################################################################


# Use our App Engine service account's credentials.
EE_CREDENTIALS = ee.ServiceAccountCredentials(
    config.EE_ACCOUNT, config.EE_PRIVATE_KEY_FILE)

# Create the Jinja templating system we use to dynamically generate HTML. See:
# http://jinja.pocoo.org/docs/dev/
JINJA2_ENVIRONMENT = jinja2.Environment(
    loader=jinja2.FileSystemLoader(os.path.dirname(__file__)),
    autoescape=True,
    extensions=['jinja2.ext.autoescape'])

# Initialize the EE API.
ee.Initialize(EE_CREDENTIALS)
