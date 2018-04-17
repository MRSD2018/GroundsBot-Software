class Region(ndb.Model):
  region = ndb.StringProperty(indexed=False)
  
class Path(ndb.Model):
  path = ndb.StringProperty(indexed=False)
  
class Approval(ndb.Model):
  approval = ndb.StringProperty(indexed=False)

