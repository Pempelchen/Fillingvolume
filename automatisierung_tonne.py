import os, PhotoScan, datetime, sys, json

def get_barcode():
    fileUri = sys.argv[1]
    nameSplit = fileUri.split('_')
    barcode = nameSplit[3]
    return barcode

def save_project(barcode, path_projekt): 
    #path= path_projekt
    fileName = get_date() + '_' + datetime.datetime.now().strftime("%f") + '_' + barcode
    file = path_projekt + fileName
    if file[-4:].lower() != ".psx":
        file += ".psx"
    doc.save(file)
    return fileName 
    
def add_photos(chunk, barcode, path_server):
    photos = os.listdir(path_server)
    project_photos=[]
    for photo_name in photos:
        if photo_name != 'volumen.json':
            name_split = photo_name.split("_")
            if name_split[2] == barcode:
                project_photos.append(path_server + photo_name)
    chunk.addPhotos(project_photos)
    return project_photos
    # photos_path=[]
    # for photo_name in project_photos:
    #     photos_path.append(path + photo_name)
    # chunk.addPhotos(photos_path)
    # return photos_path

def get_marker_reference(chunk, path_koordinaten):
	chunk.loadReference(path_koordinaten + 'marker_koordinaten.csv', columns= "nxyz", delimiter=";")

	chunk.updateTransform()

def scale_bar(chunk, path_koordinaten):
    file = open(path_koordinaten + 'marker_distance.csv', "rt")
	
    eof = False
    line = file.readline()
    scale1 = chunk.markers[1].label
    scale2 = chunk.markers[2].label
    while not eof:
        point1, point2, dist = line.split(";")	
        for marker in chunk.markers:
            counter = 0
            if marker.label == 'target ' + point1:
                scale1 = marker
                counter += 1
            elif marker.label == 'target ' +  point2:
                scale2 = marker
                counter += 1
        if counter == 2:
            if 'target ' + point1 != scale1.label:
                newmarker = chunk.addMarker()
                newmarker.label =  'target ' + point1
                scale1 = newmarker
            if 'target ' + point2 != scale2.label:
                newmarker = chunk.addMarker()
                newmarker.label =  'target ' + point2
                scale2 = newmarker

            scalebar = chunk.addScalebar(scale1, scale2)
            scalebar.reference.distance = float(dist)
        line = file.readline()		#reading the line in input file
        if not len(line):
            eof = True
            break	
    file.close()
    PhotoScan.app.update()    

def optimize_cameras(chunk):
	chunk.optimizeCameras(fit_f=True, fit_cx=True, fit_cy=True, fit_b1=False, fit_b2=False, fit_k1=True, fit_k2=True, fit_k3=True, fit_k4=False, fit_p1=True, fit_p2=True, fit_p3=False,fit_p4=False, adaptive_fitting=False)

def camera_calibration(chunk, path_projekt):
    path = path_projekt + 'kalibrierung.xml'
    s = chunk.sensors[0]
    s.user_calib = PhotoScan.Calibration()
    c = PhotoScan.Calibration()
    c.load(path)
    s.user_calib = c
    s.fixed  = True

def align_photos(chunk):
    #LowestAccuracy, LowAccuracy, MediumAccuracy, MediumAccuracy, HighAccuracy, HighestAccuracy
    chunk.matchPhotos(accuracy=PhotoScan.MediumAccuracy, generic_preselection= True, reference_preselection=False, keypoint_limit=70000,)
    chunk.alignCameras()

def local_coordinates(chunk):
	chunk.crs = PhotoScan.CoordinateSystem('LOCAL_CS["Local CS",LOCAL_DATUM["Local Datum",0],UNIT["metre",1]]')


def build_dense_cloud(chunk):
	chunk.buildDepthMaps(quality = PhotoScan.MediumQuality, filter = PhotoScan.AggressiveFiltering)#LowestQuality, LowQuality, MediumQuality, HighQuality, UltraQuality
	chunk.buildDenseCloud()

def detect_markers(chunk):
	chunk.detectMarkers(type=PhotoScan.CircularTarget12bit)

def build_dem(chunk):
	chunk.buildDem(source=PhotoScan.DataSource.DenseCloudData, interpolation=PhotoScan.Interpolation.EnabledInterpolation)

def bounding_box(chunk):
    CORNERS = ["target 18", "target 5", "target 10", "target 4"]
    # target 5----------target 10
    #     |               |
    #     |               |
    # target 18---------target 4
    # x,y-plane
    def vect(a, b):
            
        result = PhotoScan.Vector([a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y *b.x])
        return result.normalized()
            
    def get_marker(label, chunk):
        
        for marker in chunk.markers:
            if marker.label.lower() == label.lower():
                return marker
        return None
        
    doc = PhotoScan.app.document
    chunk = doc.chunk #active chunk
    if chunk.transform.matrix:
        T = chunk.transform.matrix
        s = chunk.transform.scale
    else:
        T = PhotoScan.Matrix().Diag([1,1,1,1])
        s = 1
        
    for x in CORNERS:
        print('get_marker: ', get_marker(x, chunk))
        if get_marker(x, chunk) == None:
            return
    points2 = [get_marker(x, chunk).position for x in CORNERS]  #enthält die Marker positionen
  
    counter = 0
    for x in points2:
        if x:
            counter += 1
    print("Counter: ", counter)
    print("1Points2 Vektor", points2)#
    if counter == 4:
        print("2Points2 Vektor", points2)#
        new_region = chunk.region
        new_center = (points2[0] + points2[1] + points2[2] + points2[3]) / 4.
                                
        side1 = points2[0] - points2[1] 
        side2 = points2[0] - points2[-1] 
        side1g = T.mulp(points2[0]) - T.mulp(points2[1])
        side2g = T.mulp(points2[0]) - T.mulp(points2[-1])
                                
        new_size = PhotoScan.Vector([side2g.norm()/s *1.4, side1g.norm()/s * 1.4, new_region.size.z * 2.5])
                                
        horizontal = side2
        vertical = side1
        normal = vect(vertical, horizontal) #Kreuzprodunkt: Vektor der senkrecht auf vertical, horizontal steht, Länge 1
        horizontal = -vect(vertical, normal)
        vertical = vertical.normalized()
                                
        R = PhotoScan.Matrix ([horizontal, vertical, -normal])
        new_region.rot = R.t()
        new_region.center = new_center
        new_region.size = new_size

def measure_volume(chunk, barcode, fileName, path_tonnenrand, path_volumen):
    chunk.importShapes(path_tonnenrand)
    custom = 0
    method = custom   #'bestfit', 'mean'
    for shape in chunk.shapes:
        volume = shape.volume(level = method)
        print(volume)
        print('below: ', volume['below'])
    datei = open(path_volumen, 'a')
    datei.write('\n' + barcode + '\t' + fileName + '\t' + str(240 - volume['below'] * 1000))
    datei.close()
    volume_json(volume['below'], barcode)

def volume_json(volume, barcode):
    if os.path.exists(path_server + 'volumen.json'):
        with open(path_server + 'volumen.json') as f:
            data = json.load(f)  
    else:
        data = {}  
        data['volumen'] = []
    date = get_date()
    if volume >= 0:
        volumeL = round(240 - volume * 1000)
    else:
        volumeL = 'Error'
    data['volumen'].append({
        'barcode': barcode, 'vol': volumeL, 'date': date
    })
    with open(path_server + 'volumen.json', 'w') as outfile:
        outfile.write(json.dumps(data, indent=4, sort_keys=True))

def get_date():
    date= datetime.date.today().strftime("%Y") + datetime.date.today().strftime("%m") + datetime.date.today().strftime("%d")  
    print('Date: ',date)
    return date


doc = PhotoScan.app.document
#Dateipfade:
path_server = 'W:server/images/'
path_projekt = 'Y:/StudentischeProjekte/MA_arbeiter/Projekte_Tonne/'
path_koordinaten = 'Y:/StudentischeProjekte/MA_arbeiter/Automatisierung/'
path_tonnenrand = 'Y:/StudentischeProjekte/MA_arbeiter/Agisoft_Dateien/tonne_poly.shp'
path_volumen = 'Y:/StudentischeProjekte/MA_arbeiter/Projekte_Tonne/volumen.csv'
#############

barcode = get_barcode()
fileName = save_project(barcode, path_projekt)  
chunk = doc.chunk
    
doc.save()
chunk = doc.addChunk()

photos = add_photos(chunk, barcode, path_server)
camera_calibration(chunk, path_projekt)
detect_markers(chunk)
align_photos(chunk)
doc.save()
bounding_box(chunk)
doc.save()
get_marker_reference(chunk, path_koordinaten)
scale_bar(chunk, path_koordinaten)
local_coordinates(chunk)
doc.save()
optimize_cameras(chunk)
align_photos(chunk)

build_dense_cloud(chunk)
doc.save()
build_dem(chunk)
measure_volume(chunk, barcode, fileName, path_tonnenrand, path_volumen)

doc.save()