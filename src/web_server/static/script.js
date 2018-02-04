/**
 * @fileoverview Runs the grudsby application. The code is executed in the
 * user's browser. It communicates with the App Engine backend, renders output
 * to the screen, and handles user interactions.
 */

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

  // Register a click handler to toggle polygon drawing mode.
  $('.approvePlan').click(this.approvePlan.bind(this));
  
  // Timer for loading the mowing plan
  this.checkPlan();
  var timer = setInterval(this.checkPlan.bind(this),1000);

  // Write the unapproved state at the start of loading the page.
  this.loadApproval();
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
        fillColor: 'white',
        strokeColor: 'white',
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
    $('.drawRegion').get(0).innerHTML="Draw New Region";
    if (grudsby.App.currentRegion!=null){
      grudsby.App.currentRegion.setEditable(true);
    };
  }
  else
  {
    grudsby.App.polyDrawingManager.setOptions({
      drawingMode: google.maps.drawing.OverlayType.POLYGON
    });
    $('.drawRegion').get(0).innerHTML='Edit Current Region';
    if (grudsby.App.currentRegion!=null){
      grudsby.App.currentRegion.setEditable(false);
    };
  };
};

grudsby.App.prototype.regionComplete = function(polygon) {
  grudsby.App.polyDrawingManager.setOptions({
    drawingMode: null
  });
  $('.drawRegion').get(0).innerHTML="Draw New Region";
  if (grudsby.App.currentRegion!=null){
    grudsby.App.currentRegion.setMap(null);
  };
  polygon.setEditable(true);
  grudsby.App.currentRegion = polygon;
  this.planInvalidated();
  var polyPath = polygon.getPath()
  google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
  google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));
  
};

grudsby.App.prototype.approvePlan = function() {
  if ((grudsby.App.planApproved == false) && (grudsby.App.currentRegion!=null)) {
    $('.approvePlan').get(0).innerHTML="Mowing Plan Approved";
    $('.approvePlan').get(0).setAttribute("style","background-color: #4CAF50");
    grudsby.App.planApproved = true;
    this.saveApproval();
  }
  else {
    this.planInvalidated();
  };
};

grudsby.App.prototype.planInvalidated = function() {
  $('.approvePlan').get(0).innerHTML="Approve Mowing Plan";
  $('.approvePlan').get(0).setAttribute("style","background-color: #f44336");
  grudsby.App.planApproved = false;
  this.saveApproval();
  this.savePolygon();
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
        fillColor: 'white',
        strokeColor: 'white',
        strokeWeight: 3,
        map: this.map,
        editable: true
      });
      var polyPath = grudsby.App.currentRegion.getPath()
      google.maps.event.addListener(polyPath, 'set_at', this.polyMoved.bind(this));
      google.maps.event.addListener(polyPath, 'insert_at', this.polyMoved.bind(this));
    };
  }).bind(this)); 
};

grudsby.App.prototype.savePolygon = function() {
  var dataOut = { 
      "coordinates":grudsby.App.currentRegion.getPath().getArray(),
      "regionID":"sve"
    };          

  $.get('/saveRegion?jsonData=' + JSON.stringify(dataOut)).done((function(data) {
      if (data['error']) 
      {
        console.log("Failure: %s", data.error);
      }
      else 
      {
        
      };
    }).bind(this));

};

grudsby.App.prototype.polyMoved = function() {
  this.planInvalidated();
}

grudsby.App.prototype.checkPlan = function() {
  polygonId = "sve";
  $.get('/plan?polygon_id=' + polygonId).done((function(data) {
    if (data['error']) 
    {
      console.log("Failure: %s", data.error);
    }
    else 
    {
      if (grudsby.App.currentPlan!=null)
      {
        grudsby.App.currentPlan.setMap(null);
      };
      grudsby.App.currentPlan = new google.maps.Polyline({
        path: data.coordinates,
        strokeColor: 'red',
        strokeWeight: 2,
        strokeOpacity: 1.0,
        map: this.map,
        editable: false
      });
    };
  }).bind(this)); 
}

grudsby.App.prototype.saveApproval = function() {
  var dataOut = { 
      "approval":"false",
      "regionID":"sve"
    };          
  if (grudsby.App.planApproved)
  {
    dataOut.approval = "true";
  };

  $.get('/setApproval?jsonData=' +JSON.stringify(dataOut)).done((function(data) {
    if (data['error']) 
    {
      console.log("Failure: %s", data.error);
    }
    else 
    {
      
    };
  }).bind(this));
}

grudsby.App.prototype.loadApproval = function() {
  $.get('/approval?polygon_id=sve').done((function(data) {
    if (data['error']) 
    {
      console.log("Failure: %s", data.error);
    }
    else 
    {
      grudsby.App.planApproved = false;
      if (data.approval == "true")
      { 
        grudsby.App.planApproved = true;
        $('.approvePlan').get(0).innerHTML="Mowing Plan Approved";
        $('.approvePlan').get(0).setAttribute("style","background-color: #4CAF50");
      }
    };
  }).bind(this)); 
}


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
grudsby.App.DEFAULT_ZOOM = 20;


/** @type {Object} The default center of the map. */
grudsby.App.DEFAULT_CENTER = {lng: -79.940813, lat: 40.444522};

/** @type {Object} The polygon drawing manager. */
grudsby.App.polyDrawingManager;


/** @type {Object} The current polygon. */
grudsby.App.currentRegion;

/** @type {Object} The current polygon. */
grudsby.App.currentPlan;

/** @type {Object} Tracks whether the user has approved the region. */
grudsby.App.planApproved = false;

