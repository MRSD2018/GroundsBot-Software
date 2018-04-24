/**
 * @fileoverview Runs the grudsby application. The code is executed in the
 * user's browser. It communicates with the App Engine backend, renders output
 * to the screen, and handles user interactions.
 */

var RotateIcon = function(options){
    this.options = options || {};
    this.rImg = options.img || new Image();
    this.rImg.src = this.rImg.src || this.options.url;
    this.options.width = this.options.width || this.rImg.width || 52;
    this.options.height = this.options.height || this.rImg.height || 60;
    canvas = document.createElement("canvas");
    canvas.width = this.options.width;
    canvas.height = this.options.height;
    this.context = canvas.getContext("2d");
    this.canvas = canvas;
};
RotateIcon.makeIcon = function(url) {
    
    
    return new RotateIcon({url: url});
};
RotateIcon.prototype.setRotation = function(options){
    var canvas = this.context,
        angle = options.deg ? options.deg * Math.PI / 180:
            options.rad,
        centerX = this.options.width/2,
        centerY = this.options.height/2;

    canvas.clearRect(0, 0, this.options.width, this.options.height);
    canvas.save();
    canvas.translate(centerX, centerY);
    canvas.rotate(angle);
    canvas.translate(-centerX, -centerY);
    canvas.drawImage(this.rImg, 0, 0);
    canvas.restore();
    return this;
};

RotateIcon.prototype.getUrl = function(){
    return this.canvas.toDataURL('image/png');
};

grudsby = {};  // Our namespace.

grudsby.boot = function(eeMapId, eeToken) {
  // Load external libraries.
  google.load('visualization', '1.0');
  google.load('jquery', '1');
  google.load('maps', '3', {'other_params': 'libraries=drawing&key=AIzaSyCY0S3eiXE_-2iWY23mN-LRq8YDV57B_us'});

  // Create the grudsby app.
  google.setOnLoadCallback(function() {
    var mapType = grudsby.App.getEeMapType(eeMapId, eeToken);
    var app = new grudsby.App(mapType);
  });
};


///////////////////////////////////////////////////////////////////////////////
//                               The application.                            //
///////////////////////////////////////////////////////////////////////////////


grudsby.App = function(mapType) {
  // Create and display the map.
  this.map = this.createMap(mapType);


  // Add the polygons to the map.
  this.addPolygon();

  // Register a click handler to toggle polygon drawing mode.
  $('.drawRegion').click(this.drawRegion.bind(this));

  // Add the polygons to the map.

  grudsby.grudsby_lat = 0.0;
  grudsby.grudsby_lng = 0.0; 

  // Register a click handler to add an obstacle.
  $('.addObstacle').click(this.addObstacle.bind(this));

  // Register a click handler to clear the obstacles.
  $('.clearObstacles').click(this.clearObstacles.bind(this));
 


 
  var marker = new google.maps.Marker({
      position: {lat: 40.444505, lng: -79.940777},
      map: this.map,
      title: 'grudsby',
      zIndex:400
    });
  var step = 1;
  var angle = 1;

  setInterval(function(){
    $.get('/mowerPos').done((function(data) {
      if (data['error']) 
      {
        console.log("Failure: %s", data.error);
      }
      else 
      {

        grudsby.grudsby_lat = data.lat;
        grudsby.grudsby_lng = data.lng; 

        $('.grudsby').attr('src', RotateIcon.makeIcon("/static/grudsby_top_tiny.png")
            .setRotation({deg: -data.rot})
            .getUrl());
        marker.setOptions({
            icon: {
              url:$('.grudsby').attr('src'),
              size: new google.maps.Size(80, 80),
              origin: new google.maps.Point(0, 0),
              anchor: new google.maps.Point(40, 40),
              scaledSize: new google.maps.Size(80, 80) },
            position: {lat: data.lat, lng: data.lng},

        });
      };
    }).bind(this));                           
  }, 1000);


  
  $.get('/obstacles').done((function(data) {
    if (data['error']) 
    {
      console.log("Failure: %s", data.error);
    }
    else 
    {
      for (i = 0; i < data['obstacles'].length; i++) { 
        obst = data['obstacles'][i];        
        var marker = new google.maps.Marker({
          position: {lat: obst['lat'], lng: obst['lng']},
          map: this.map,
          title: 'obstacle',
          zIndex: 300,
        });
        marker.setOptions({
          icon: {
            url:"/static/some_beach_tiny.png",
            size: new google.maps.Size(20, 20),
            origin: new google.maps.Point(0, 0),
            anchor: new google.maps.Point(10, 10),
            scaledSize: new google.maps.Size(20, 20) },
        });
      }
    };
  }).bind(this)); 
};


grudsby.App.prototype.createMap = function(mapType) {
  var mapOptions = {
    center: grudsby.App.DEFAULT_CENTER,
    disableDefaultUI: true,
    zoom: grudsby.App.DEFAULT_ZOOM
  };
  var mapEl = $('.map').get(0);
  var map = new google.maps.Map(mapEl, mapOptions);
  //map.overlayMapTypes.push(mapType); // For adding overlays
  map.setMapTypeId('satellite');
  grudsby.App.polyDrawingManager = new google.maps.drawing.DrawingManager({
      drawingMode: null,
      drawingControl: false,
      map: map,
      polygonOptions: {
        fillColor: 'blue',
        strokeColor: 'blue',
	editable: false
      }
    });
  google.maps.event.addListener(grudsby.App.polyDrawingManager, 'polygoncomplete', this.regionComplete.bind(this));
  return map;

};

grudsby.App.prototype.drawRegion = function() {
  if (grudsby.App.polyDrawingManager.getDrawingMode() == google.maps.drawing.OverlayType.POLYGON)
  {
    grudsby.App.polyDrawingManager.setOptions({
      drawingMode: null
    });
    $('.drawRegion').get(0).innerHTML="Draw New Geofence";
    if (grudsby.App.currentRegion!=null){
      grudsby.App.currentRegion.setEditable(true);
    };
  }
  else
  {
    grudsby.App.polyDrawingManager.setOptions({
      drawingMode: google.maps.drawing.OverlayType.POLYGON
    });
    $('.drawRegion').get(0).innerHTML='Edit Current Geofence';
    if (grudsby.App.currentRegion!=null){
      grudsby.App.currentRegion.setEditable(false);
    };
  };
};

grudsby.App.prototype.regionComplete = function(polygon) {
  grudsby.App.polyDrawingManager.setOptions({
    drawingMode: null
  });
  $('.drawRegion').get(0).innerHTML="Draw New Geofence";
  if (grudsby.App.currentRegion!=null){
    grudsby.App.currentRegion.setMap(null);
  };
  polygon.setEditable(true);
  polygon.setOptions({
    zIndex:10
  });
  grudsby.App.currentRegion = polygon;
  this.planInvalidated();
  var polyPath = polygon.getPath()
  google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
  google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));

};

grudsby.App.prototype.addPolygon = function() {
  polygonId = "sve";
  $.get('/region?polygon_id=' + polygonId).done((function(data) {
    if (data['error'])
    {
      console.log("Failure: %s", data.error);
    }
    else
    {
      grudsby.App.currentRegion = new google.maps.Polygon({
        paths: data.coordinates,
        fillColor: 'blue',
        strokeColor: 'blue',
        strokeWeight: 3,
        map: this.map,
        editable: true,
        zIndex: 10
      });
      var polyPath = grudsby.App.currentRegion.getPath()
      google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
      google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));
    };
  }).bind(this));
};

grudsby.App.prototype.planInvalidated = function() {
  this.savePolygon();
};

grudsby.App.prototype.polyMoved = function() {
  this.planInvalidated();
}

grudsby.App.prototype.removeDuplicates = function() {
  var path_to_snap = grudsby.App.currentRegion.getPath().getArray();
  var path_list = [];
  for (var i = 0; i < path_to_snap.length; i++) {
    path_list.push({lng:path_to_snap[i].lng(),lat:path_to_snap[i].lat()});
  };

  var i = 0; 
  continue_loop = true; 
  do {
    var new_path_list = JSON.parse(JSON.stringify(path_list));
    new_path_list.splice(i,1);
    if (this.isDuplicatePoint(path_list[i], new_path_list)) {
      path_list.splice(i,1);
    }
    else {
      i += 1;
    }
    continue_loop = (i < path_list.length);
  } while (continue_loop); 
 
  var new_path_latlng = [];
  for (var i = 0; i < path_list.length; i++) {
    new_path_latlng.push(new google.maps.LatLng(path_list[i]));
  };

  grudsby.App.currentRegion.setPath(new_path_latlng); 
  var polyPath = grudsby.App.currentRegion.getPath()
  google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
  google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));
    
};


grudsby.App.prototype.enforceConvexity = function() {
  var path_to_snap = grudsby.App.currentRegion.getPath().getArray();
  if (path_to_snap.length >3) {
    //path_to_snap.pop();
    var path_list = [];
    for (var i = 0; i < path_to_snap.length; i++) {
      path_list.push({lng:path_to_snap[i].lng(),lat:path_to_snap[i].lat()});
    };

    var i = 0; 
    continue_loop = true; 
    do {
      var new_path_list = JSON.parse(JSON.stringify(path_list));
      new_path_list.splice(i,1);
      new_path_list.push(new_path_list[0]);
      
      if (this.isInGeofence(path_list[i], new_path_list)) {
        path_list.splice(i,1);
      }
      else {
        i += 1;
      }
      continue_loop = ((i < path_list.length) && (path_list.length > 3));
    } while (continue_loop); 
    
    var new_path_latlng = [];
    for (var i = 0; i < path_list.length; i++) {
      new_path_latlng.push(new google.maps.LatLng(path_list[i]));
    };
  
    grudsby.App.currentRegion.setPath(new_path_latlng); 
    var polyPath = grudsby.App.currentRegion.getPath()
    google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
    google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));
  }   
};

grudsby.App.prototype.isDuplicatePoint = function(pt, polygon) {
  for (var j = 0; j < (polygon.length-1); j++) {
    if (Math.sqrt((polygon[j].lat-pt.lat)**2.0 + (polygon[j].lng-pt.lng)**2.0) < 0.000005) {
      return true;
    }
  }
  return false;
}

grudsby.App.prototype.isInGeofence = function(pt, geofence) {
  var wn = 0;
  for (var j = 0; j < (geofence.length-1); j++) {
    if (geofence[j].lat <= pt.lat) {
      if (geofence[j+1].lat > pt.lat) {
        if (this.isLeft(geofence[j],geofence[j+1], pt) > 0) {
          wn++;
        }
      }
    }
    else {
      if (geofence[j+1].lat <= pt.lat) {
        if (this.isLeft(geofence[j],geofence[j+1], pt) < 0) {
          wn--;
        } 
      }
    }
  }
  return (wn != 0);
};

grudsby.App.prototype.isLeft = function(P0, P1, P2) {
  return ( (P1.lng - P0.lng) * (P2.lat - P0.lat) - (P2.lng - P0.lng) * (P1.lat - P0.lat));
};


grudsby.App.prototype.savePolygon = function() {
  this.enforceConvexity();
  this.removeDuplicates();
  var dataOut = {
      "coordinates":grudsby.App.currentRegion.getPath().getArray(),
      "regionID":"sve"
    };

  $.get('/saveGeofence?jsonData=' + JSON.stringify(dataOut)).done((function(data) {
      if (data['error'])
      {
        console.log("Failure: %s", data.error);
      }
      else
      {

      };
    }).bind(this));

};

grudsby.App.prototype.addPolygon = function() {
  polygonId = "sve";
  $.get('/geofence?polygon_id=' + polygonId).done((function(data) {
    if (data['error'])
    {
      console.log("Failure: %s", data.error);
    }
    else
    {
      grudsby.App.currentRegion = new google.maps.Polygon({
        paths: data.coordinates,
        fillColor: 'blue',
        strokeColor: 'blue',
        strokeWeight: 3,
        map: this.map,
        editable: true,
        zIndex: 10
      });
      var polyPath = grudsby.App.currentRegion.getPath()
      google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
      google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));
    };
  }).bind(this));
};

grudsby.App.prototype.addObstacle = function() {

  var dataOut = { 
      lat:grudsby.grudsby_lat,
      lng:grudsby.grudsby_lng
    };   
  //console.log("strin: %s", JSON.stringify(dataOut));       
  $.get('/addObstacle?jsonData=' + JSON.stringify(dataOut)).done((function(data) {
      if (data['error']) 
      {
        console.log("Failure: %s", data.error);
      }
      else 
      {
         var marker = new google.maps.Marker({
          position: {lat: grudsby.grudsby_lat, lng: grudsby.grudsby_lng},
          map: this.map,
          title: 'obstacle',
          zIndex: 300,
        });
        marker.setOptions({
          icon: {
            url:"/static/some_beach_tiny.png",
            size: new google.maps.Size(20, 20),
            origin: new google.maps.Point(0, 0),
            anchor: new google.maps.Point(10, 10),
            scaledSize: new google.maps.Size(20, 20) },
        });
      };
    }).bind(this));
};

grudsby.App.prototype.clearObstacles = function() {
    $.get('/clearObstacles')
  };

///////////////////////////////////////////////////////////////////////////////
//                        Static helpers and constants.                      //
///////////////////////////////////////////////////////////////////////////////


grudsby.App.getEeMapType = function(eeMapId, eeToken) {
  var eeMapOptions = {
    getTileUrl: function(tile, zoom) {
      var url = grudsby.App.EE_URL + '/map/';
      url += [eeMapId, zoom, tile.x, tile.y].join('/');
      url += '?token=' + eeToken;
      return url;
    },
    tileSize: new google.maps.Size(256, 256)
  };
  return new google.maps.ImageMapType(eeMapOptions);
};

/** @type {string} The Earth Engine API URL. */
grudsby.App.EE_URL = 'https://earthengine.googleapis.com';



/** @type {number} The default zoom level for the map. */
grudsby.App.DEFAULT_ZOOM = 21;


/** @type {Object} The default center of the map. */
grudsby.App.DEFAULT_CENTER = {lng: -79.940763, lat: 40.444546};

/** @type {Object} The polygon drawing manager. */
grudsby.App.polyDrawingManager;


/** @type {Object} The current polygon. */
grudsby.App.currentRegion;


grudsby.App.currentPlanText = "";
 

/** @type {Object} The current polygon. */
grudsby.App.currentPlan;

/** @type {Object} Tracks whether the user has approved the region. */
grudsby.App.planApproved = false;

