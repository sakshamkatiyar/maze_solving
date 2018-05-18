import numpy as np
import cv2
import copy

## Reads image in HSV format. Accepts filepath as input argument and returns the HSV
## equivalent of the image.
def readImageHSV(filePath):
    #############  Add your Code here   ###############
    img = cv2.imread(filePath)                                      ## Read image
    hsvImg = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                   ## Convert image to hsv
    ###################################################
    return hsvImg

## Reads image in binary format. Accepts filepath as input argument and returns the binary
## equivalent of the image.
def readImageBinary(filePath):
    #############  Add your Code here   ###############
    img = cv2.imread(filePath)                                      ## Read image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)                    ## Convert image to grayscale image
    ret, binaryImage = cv2.threshold(gray, 153, 255, 0)             ## Convert image to Binary
    ###################################################
    return binaryImage

## The findNeighbours function takes a maze image and row and column coordinates of a cell as input arguments
## and returns a stack consisting of all the neighbours of the cell as output.
## Note :- Neighbour refers to all the adjacent cells one can traverse to from that cell provided only horizontal
## and vertical traversal is allowed.
def findNeighbours(img,row,column):
    neighbours = []
    #############  Add your Code here   ###############
    if img[20*row+10, 20*column+1]>=51:                             ## Check the left edge pixel value
        neighbours.append((row, column-1))                          ## If true decrease column value and return left cell coordinate
    if img[20*row+1, 20*column+10]>=51:                             ## Check the upper edge pixel value
        neighbours.append((row-1, column))                          ## If true decrease row value and return upper cell coordinate
    if img[20*row+19, 20*column+10]>=51:                            ## Check the lower edge pixel value
        neighbours.append((row+1, column))                          ## If true increase row value and return lower cell coordinate
    if img[20*row+10, 20*column+19]>=51:                            ## Check the right edge pixel value
        neighbours.append((row, column+1))                          ## If true increase column value and return right cell coordinate
    ###################################################
    return neighbours

##  colourCell basically highlights the given cell by painting it with the given colourVal. Care should be taken that
##  the function doesn't paint over the black walls and only paints the empty spaces. This function returns the image
##  with the painted cell.
##  You can change the colourCell() functions used in the previous sections to suit your requirements.

def colourCell(img,row,column,colourVal):   ## Add required arguments here.
    
    #############  Add your Code here   ###############
    for i in range(20*row,20*row+20):                               ## Specify row range of a cell to be highlighted
        for j in range(20*column,20*column+20):                     ## Specify column range of a cell to be highlighted
            if img[i,j]==255:                                       ## Only empty white spaces with value=255 are highlighted
                img[i,j]=colourVal
    ###################################################
    return img

##  Function that accepts some arguments from user and returns the graph of the maze image.
def buildGraph(img):  ## You can pass your own arguments in this space.
    graph = {}
    #############  Add your Code here   ###############
    last_point = (len(img)/20-1,len(img)/20-1)                      ## Get coordinates of top right cell
    for i in range(0,last_point[0]+1):                              ## Range of i(abscissa) from 0 to abscissa of last_point
        for j in range(0,last_point[1]+1):                          ## Range of j(ordinate) from 0 to ordinate of last point
            neighbours = findNeighbours(img, i, j)                  ## Call function findNeignbours to find neighbours
            graph[(i, j)] = neighbours                              ## Returns coordinates of neighbours to graph string
    ###################################################

    return graph

##  Finds shortest path between two coordinates in the maze. Returns a set of coordinates from initial point
##  to final point.
def findPath(graph, start, end, path=[]): ## You can pass your own arguments in this space.
    #############  Add your Code here   ###############
    path = path + [start]                                           ## Dijkstra's algorithm to find shortest between nodes in the graph
    if start == end:                                                
        return path                                                 
    if not graph.has_key(start):                                    
        return path                                                 
    shortest = None                                                 
    for node in graph[start]:                                       
        if node not in path:                                        
            newpath = findPath(graph, node, end, path)              
            if newpath:                                             
                if not shortest or len(newpath) < len(shortest):    
                    shortest = newpath
    ###################################################
    return shortest     

## The findMarkers() function returns a list of coloured markers in form of a python dictionaries
## For example if a blue marker is present at (3,6) and red marker is present at (1,5) then the
## dictionary is returned as :-
##          list_of_markers = { 'Blue':(3,6), 'Red':(1,5)}

def findMarkers(img,):    ## You can pass your own arguments in this space.
    list_of_markers = {}
    #############  Add your Code here   ###############
    bgr = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    im = copy.copy(bgr)                                                                             ## Converting the image to grayscale
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray,153,255,0)                                                     ## Obtaining binary image
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for i in range(1,len(contours)):                                                                ## Skipping the first contour as it the whole grid
        M = cv2.moments(contours[i])                                                                ## Moment of contour i
        cx = int(M['m10']/M['m00'])                                                                 ## X coordinate of centroid of contour i
        cy = int(M['m01']/M['m00'])                                                                 ## Y coordinate of centroid of contour i
        value = check(im, cx, cy)
        if value!=0:
            list_of_markers.update(value)
    ###################################################
    return list_of_markers

## The findOptimumPath() function returns a python list which consists of all paths that need to be traversed
## in order to start from the bottom left corner of the maze, collect all the markers by traversing to them and
## then traverse to the top right corner of the maze.

def findOptimumPath(img, listOfMarkers, initial_point, final_point):     ## You can pass your own arguments in this space.
    path_array = []
    #############  Add your Code here   ###############
    graph = buildGraph(img)
    path_array = graph
    for k1, v1 in listOfMarkers.iteritems():                    ## Grab the coordinates of a markers from list of markers
        p1 = [findPath(graph, initial_point, v1)]               ## find shortest path from initial point to the marker point
        for k2, v2 in listOfMarkers.iteritems():                ## Grab another marker from list of markers
            if v1!=v2:                                          ## Check if the markers are not same
                p1+=[findPath(graph, v1, v2)]                   ## Add path from previous marker to next marker
        p1+=[findPath(graph, v2, final_point)]                  ## find shortest path from marker point to final point
        if(len(p1)<len(path_array)):                        
            path_array = []                             
            path_array+=p1                                      ## Add total paths to be traversed by starting from bottom left, collecting all markers and reaching top right corner
    ###################################################
    return path_array
        
## The colourPath() function highlights the whole path that needs to be traversed in the maze image and
## returns the final image.

def colourPath(img, pathArray):                                 ## You can pass your own arguments in this space. 
    #############  Add your Code here   ###############
    for path in pathArray:                                      ## Loop to paint the solution path.
        for i in path:
            img = colourCell(img, i[0], i[1], 200)
    ###################################################
    return img

#####################################    Add Utility Functions Here   ###################################
##                                                                                                     ##
##                   You are free to define any functions you want in this space.                      ##
##                             The functions should be properly explained.                             ##
def check(img, x, y):
    b, g, r = img[y, x]
    if b in range(230,256) and g in range(0,26) and r in range(0,26):           ## RGB range for Blue coloured cell
        return {'Blue' :(y/20,x/20)}                                            ## Retun cell co-ordinates of Blue cell
    elif b in range(0,26) and g in range(230,256) and r in range(0,26):         ## RGB range for Green coloured cell
        return {'Green' :(y/20,x/20)}                                           ## Retun cell co-ordinates of Green cell
    elif b in range(0,26) and g in range(0,26) and r in range(230,256):         ## RGB range for Red coloured cell
        return {'Red' :(y/20,x/20)}                                             ## Retun cell co-ordinates of Red cell
    elif b in range(230,256) and g in range(0,26) and r in range(230,256):      ## RGB range for Pink coloured cell
        return {'Pink' :(y/20,x/20)}                                            ## Retun cell co-ordinates of Pink cell
    return 0
##                                                                                                     ##
##                                                                                                     ##
#########################################################################################################

## This is the main() function for the code, you are not allowed to change any statements in this part of
## the code. You are only allowed to change the arguments supplied in the findMarkers(), findOptimumPath()
## and colourPath() functions.

def main(filePath, flag = 0):
    imgHSV = readImageHSV(filePath)                ## Acquire HSV equivalent of image.
    listOfMarkers = findMarkers(imgHSV)              ## Acquire the list of markers with their coordinates. 
    test = str(listOfMarkers)
    imgBinary = readImageBinary(filePath)          ## Acquire the binary equivalent of image.
    initial_point = ((len(imgBinary)/20)-1,0)      ## Bottom Left Corner Cell
    final_point = (0, (len(imgBinary[0])/20) - 1)  ## Top Right Corner Cell
    pathArray = findOptimumPath(imgBinary, listOfMarkers, initial_point, final_point) ## Acquire the list of paths for optimum traversal.
    print pathArray
    img = colourPath(imgBinary, pathArray)         ## Highlight the whole optimum path in the maze image
    if __name__ == "__main__":                    
        return img
    else:
        if flag == 0:
            return pathArray
        elif flag == 1:
            return test + "\n"
        else:
            return img
## Modify the filepath in this section to test your solution for different maze images.           
if __name__ == "__main__":
    filePath = "maze00.jpg"                        ## Insert filepath of image here
    img = main(filePath)                 
    cv2.imshow("canvas", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


