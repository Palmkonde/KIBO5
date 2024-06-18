package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import org.opencv.android.Utils;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.ximgproc.Ximgproc;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
    private final int LOOPMAX = 3;
    private final Box KIZ = new Box(new Point3D(10.3d, -10.2d, 4.32d),
                                    new Point3D(11.55d, -6.0d, 5.57d));

    private final Box[] KOZ = {
            new Box(new Point3D(10.87d ,-9.5d, 4.27d),
                    new Point3D(11.6d, -9.45d, 4.97d)),

            new Box(new Point3D(10.25d, -9.5d, 4.97d),
                    new Point3D(10.87d, -9.45d, 5.62d)),

            new Box(new Point3D(10.87d, -8.5d, 4.97d),
                    new Point3D(11.6d, -8.45d, 5.62d )),

            new Box(new Point3D(10.25d, -8.5d, 4.27d),
                    new Point3D(10.7d, -8.45d, 4.97d)),

            new Box(new Point3D(10.72d, -7.40d, 4.27d), // x - 0.15
                    new Point3D(11.6d, -7.35d, 5.02d)), // z + 0.05

            new Box(new Point3D(10.25d, -7.40d, 4.92d), // z - 0.05
                    new Point3D(10.92d, -7.35d, 5.62d)) // x + 0.05
    };

    final double directions[][] = {
            {0.01d,0.0d,0.0d},
            {-0.01d,0.0d,0.0d},
            {0.0,0.01d,0.0},
            {0.0,-0.01d,0.0},
            {0.0,0.0,0.01d},
            {0.0,0.0,-0.01d}
    };

    final Point AREA[] = {
            new Point(10.95d,-10.16d,5.2d),
            new Point(10.93d, -8.88d, 4.8d),
            new Point(10.925d,-7.925d,4.8d),
            new Point(10.4d,-6.853d,4.945d)
    };

    private final String[] TEMPLATE_NAME = {
            "beaker",
            "goggle",
            "hammer",
            "kapton_tape",
            "pipette",
            "screwdriver",
            "thermometer",
            "top",
            "watch",
            "wrench"
    };

    private final String[] TEMPLATE_FILENAME = {
            "beaker.png",
            "goggle.png",
            "hammer.png",
            "kapton_tape.png",
            "pipette.png",
            "screwdriver.png",
            "thermometer.png",
            "top.png",
            "watch.png",
            "wrench.png"
    };

    static class Pair implements Comparable<Pair> {
        double dist;
        Point3D node;

        Pair(double dist, Point3D node) {
            this.dist = dist;
            this.node = node;
        }

        @Override
        public int compareTo(Pair other) {
            return Double.compare(this.dist, other.dist);
        }

        @Override
        public String toString() {
            return this.dist + "," + this.node;
        }
    }

    static class PQ {
        private static final int INITIAL_CAPACITY = 1005;
        Pair[] Tree;
        int Tsize = 0;

        PQ() {
            Tree = new Pair[INITIAL_CAPACITY];
        }

        void push(double dist, Point3D u)
        {
            Tree[Tsize++] = new Pair(dist,u);

            int node = Tsize-1;
            Pair tmp = Tree[node];

            while(node > 0)
            {
                int p = (node-1)/2;

                if(tmp.dist >= Tree[p].dist) break;

                Tree[node] = Tree[p];
                node = p;
            }
            Tree[node] = tmp;
        }

        void pop()
        {
            if (Tsize == 0) throw new NoSuchElementException("Priority queue is empty");

            int node = 0,c;
            Tree[node] = Tree[--Tsize];

            Pair tmp = Tree[node];

            while((c = 2*node+1) < Tsize)
            {
                if(c + 1 < Tsize && (Tree[c].dist < Tree[c+1].dist)) c++;

                if(Tree[c].dist >= tmp.dist) break;

                Tree[node] = Tree[c];
                node = c;
            }
            Tree[node] = tmp;
        }

        Pair top() {
            if (Tsize == 0) throw new NoSuchElementException("Priority queue is empty");
            return Tree[0];
        }
        boolean empty()
        {
            return (Tsize == 0);
        }
    }

    static class Point3D {
        double x, y, z;

        Point3D(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            Point3D point3D = (Point3D) o;
            return Double.compare(point3D.x, x) == 0 &&
                    Double.compare(point3D.y, y) == 0 &&
                    Double.compare(point3D.z, z) == 0;
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, y, z);
        }
    }

    static class Box {
        Point3D min, max;

        Box(Point3D min, Point3D max) {
            this.min = min;
            this.max = max;
        }
    }

    // Helper class to represent a line segment
    static class LineSegment {
        Point3D start, end;

        LineSegment(Point3D start, Point3D end) {
            this.start = start;
            this.end = end;
        }
    }

    @Override
    protected void runPlan1(){
        api.startMission();
        Log.i(TAG, "Start Mission");

        // 10.95 -10.19-0.03 5.2 <-- plate area 1
        move(10.95d,-10.16d,5.2d,0,0,-1f,1f);

        Mat saveImage = api.getMatNavCam();
        api.saveMatImage(saveImage, "Test1.jpg");

        detectImage(saveImage,1);

        // moving point
        Point nowKIBO = api.getRobotKinematics().getPosition();
        List<Point> Apath = Astar(nowKIBO,new Point(10.93d,-8.88d,4.8d));
        for (Point p:Apath){
            move(p.getX(),p.getY(),p.getZ(),0f,1f,0f,1f);
        }

        //10.93 -8.88 4.33+0.03 <-- plate area 2
        move(10.93d, -8.88d, 4.8d,0f,1f,0f,1f);

        saveImage = api.getMatNavCam();
        api.saveMatImage(saveImage, "Test2.jpg");

        nowKIBO = api.getRobotKinematics().getPosition();
        Apath = Astar(nowKIBO, new Point(10.925d,-7.925d,4.8d));
        for(Point p:Apath){
            move(p.getX(),p.getY(),p.getZ(), 0f, 1f, 0f, 1f);
        }

        // plate area 3
        move(10.925d,-7.925d,4.8d,0f,1f,0f,1f);

        saveImage = api.getMatNavCam();
        api.saveMatImage(saveImage, "Test3.jpg");

        //Astar Try hard T^T
        nowKIBO = api.getRobotKinematics().getPosition();
        List<Point> pathAstar = Astar(nowKIBO,new Point(10.30d,-6.853d,4.945d));
        for(Point p : pathAstar) {
            Log.i(TAG, "Astar go to: " + p.getX() + " " + p.getY() + " " + p.getZ());
            move(p.getX(),p.getY(),p.getZ(),0f,0f,1f,0f);
        }

        // plate area 4
        move(10.4d,-6.853d,4.945d,0f,0f,1f,0f);

        saveImage = api.getMatNavCam();
        api.saveMatImage(saveImage, "Test4.jpg");

        // astronuat 11.143, -6.7607, 4.9654,  qua: 0 0 0.707 0.707
        move(11.143d,-6.7607d,4.9654d,0f,0f,0.707f,0.707f);


        api.shutdownFactory();
        System.exit(0);
    }

    private void move(double x, double y, double z, float qx, float qy, float qz, float qw) {
        String pos = String.valueOf(x) + "," + String.valueOf(y) + "," + String.valueOf(z) + ", ";
        String qua = "qua: " + String.valueOf(qx) + "," + String.valueOf(qy) + "," + String.valueOf(qz) + "," + String.valueOf(qw);
        Log.i(TAG, pos + qua);

        Point p = new Point(x,y,z);
        Quaternion q = new Quaternion(qx,qy,qz,qw);
        Result r = api.moveTo(p,q,true);

        int cnt = 0;
        while(!r.hasSucceeded() && cnt < LOOPMAX)
        {
            r = api.moveTo(p,q, true);
            cnt++;
        }
    }

    public boolean isIntersectingBox(Point3D p1, Point3D p2, Box rectangle) {
        Point3D minPoint = rectangle.min;
        Point3D maxPoint = rectangle.max;

        double x1 = p1.x, y1 = p1.y, z1 = p1.z;
        double x2 = p2.x, y2 = p2.y, z2 = p2.z;

        // Check if the line segment intersects the box in any dimension
        boolean intersectX = (Math.min(x1, x2) <= maxPoint.x && Math.max(x1, x2) >= minPoint.x);
        boolean intersectY = (Math.min(y1, y2) <= maxPoint.y && Math.max(y1, y2) >= minPoint.y);
        boolean intersectZ = (Math.min(z1, z2) <= maxPoint.z && Math.max(z1, z2) >= minPoint.z);

        // If the line segment intersects the box in all three dimensions, it intersects the box
        return intersectX && intersectY && intersectZ;
    }

    public boolean doesLineIntersectBox(LineSegment line, Box box) {
        // Calculate intersection points
        for (int i = 0; i < 3; i++) {
            double minCoord = (i == 0) ? box.min.x : (i == 1) ? box.min.y : box.min.z;
            double maxCoord = (i == 0) ? box.max.x : (i == 1) ? box.max.y : box.max.z;

            Point3D entry = getIntersection(line.start, line.end, minCoord, i);
            Point3D exit = getIntersection(line.start, line.end, maxCoord, i);

            if (isPointInBox(entry, box) || isPointInBox(exit, box)) {
                return true;
            }
        }

        return false;
    }

    public List<Point3D> rerouteLine(LineSegment line, Box[] boxes) {
        List<Point3D> path = new ArrayList<>();

        String fr = line.start.x + ", " + line.start.y + ", " + line.start.z;
        String to = line.end.x + ", " + line.end.y+ ", " + line.end.z;
        Log.i(TAG, "rerouteAroundBox: from " + fr + " to " + to);

        for (Box box : boxes) {
            if (doesLineIntersectBox(line, box)) {
                // Calculate midpoint
                Point3D midpoint = new Point3D(
                        (line.start.x + line.end.x) / 2,
                        (line.start.y + line.end.y) / 2,
                        (line.start.z + line.end.z) / 2
                );

                // Split the line into two segments via the midpoint
                LineSegment firstSegment = new LineSegment(line.start, midpoint);
                LineSegment secondSegment = new LineSegment(midpoint, line.end);

                // Add rerouted segments to the path
                path.add(midpoint);
                path.add(line.end);
                return path; // Assuming single box intersection for simplicity
            }
        }
        path.add(line.end);
        return path;
    }

    private static Point3D getIntersection(Point3D start, Point3D end, double coord, int axis) {
        double t = (coord - (axis == 0 ? start.x : axis == 1 ? start.y : start.z)) /
                ((axis == 0 ? end.x : axis == 1 ? end.y : end.z) -
                        (axis == 0 ? start.x : axis == 1 ? start.y : start.z));
        return new Point3D(
                start.x + t * (end.x - start.x),
                start.y + t * (end.y - start.y),
                start.z + t * (end.z - start.z)
        );
    }

    private static boolean isPointInBox(Point3D point, Box box) {
        return point.x >= box.min.x && point.x <= box.max.x &&
                point.y >= box.min.y && point.y <= box.max.y &&
                point.z >= box.min.z && point.z <= box.max.z;
    }

    // Not included small case yet
    // Not done yet
    // Many case to deal with
    private List<Point> Astar(Point st, Point en) {
        List<Point> path = new ArrayList<Point>();

        Point3D start = new Point3D(st.getX(),st.getY(),st.getZ());
        Point3D end = new Point3D(en.getX(),en.getY(),en.getZ());

        // check if can go
        boolean canGo = true;
        for(Box box:KOZ) {
            if(isIntersectingBox(start,end,box)) {
                canGo = false;
            }
        }
        if(canGo) {
            path.add(en);
            return path;
        }

        Log.i(TAG, "Astar: You Cannot go there!");

        // have some box is obstacle
        Set<Point3D> visited = new HashSet<>();
        HashMap<Point3D, Double> dis = new HashMap<>();
        HashMap<Point3D, Point3D> cameFrom = new HashMap<>();

        PriorityQueue<Pair> pq = new PriorityQueue<>();
        pq.add(new Pair(0,start));
        dis.put(start,0d);

        while(!pq.isEmpty()) {
            Pair topPair = pq.poll();
            double nowDist = topPair.dist;
            Point3D nowNode = topPair.node;

            boolean isValid = true;
            for(Box box : KOZ) {
                if(isIntersectingBox(nowNode,end,box)) {
                    isValid = false;
                    break;
                }
            }
            if(isValid){
                cameFrom.put(end,nowNode);
                break;
            }

            if(visited.contains(nowNode)) continue;
            visited.add(nowNode);

            for(double[] dir : directions){
                Point3D neighbor = new Point3D(
                        nowNode.x + dir[0],
                        nowNode.y + dir[1],
                        nowNode.z + dir[2]
                );
                double nxDist = distance(nowNode,neighbor);

                if(!isPointInBox(neighbor,KIZ)) continue;

                isValid = true;
                for(Box box:KOZ) {
                    if(isIntersectingBox(nowNode,neighbor,box)) {
                        isValid = false;
                        break;
                    }
                }
                if(!isValid) continue;
                if(visited.contains(neighbor)) continue;

                double newDist = dis.get(nowNode) + nxDist;
                double fCost = dis.get(nowNode) + distance(neighbor,end);

                if(dis.getOrDefault(neighbor, Double.MAX_VALUE) > newDist) {
                    dis.put(neighbor,newDist);
                    pq.add(new Pair(fCost,neighbor));
                    cameFrom.put(neighbor,nowNode);
                }
            }
        }

        // List path
        List<Point3D> conPath = new ArrayList<Point3D>();
        Point3D current = end;

        while(current != null) {
            conPath.add(current);
            current = cameFrom.get(current);
        }
        Collections.reverse(conPath);

        // optimize path
        Point3D ref = conPath.get(0);
        for(int i=1;i<conPath.size();i++) {
            for(Box box:KOZ) {
                if (isIntersectingBox(ref,conPath.get(i),box)) {
                    ref = conPath.get(i - 1);
                    path.add(new Point(ref.x, ref.y, ref.z));
                    break;
                }
            }
        }
    return path;
    }

    private double distance (Point3D p, Point3D other) {
        return Math.sqrt(Math.pow(p.x - other.x,2) + Math.pow(p.y-other.y,2) + Math.pow(p.z-other.z,2));
    }

    private Mat resizeImg (Mat img, int width) {
        int height = (int) (img.rows() * ((double) width / img.cols()));
        Mat resizeedImg = new Mat();
        Imgproc.resize(img, resizeedImg, new Size(width, height));

        return resizeedImg;
    }

    private Mat rotImg (Mat img, int angle) {
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols()/2.0, img.rows()/2.0);
        Mat rotatedMat =  Imgproc.getRotationMatrix2D(center,angle,1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());

        return rotatedImg;
    }

    private static List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points) {
        double lenght = 10; //widht 10 px
        List<org.opencv.core.Point> filteredlist = new ArrayList<>();

        // rempve multiple detections
        for(org.opencv.core.Point point: points) {
            boolean isInclude = false;
            for(org.opencv.core.Point checkPoint : filteredlist) {
                double distance = calculateDistance(point, checkPoint);

                if(distance <= lenght){
                    isInclude = true;
                    break;
                }
            }

            if(!isInclude){
                filteredlist.add(point);
            }
        }
        return filteredlist;
    }

    private static double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2){
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(Math.pow(dx,2) + Math.pow(dy,2));
    }

    private int getMaxIndex(int[] array) {
        int max = 0;
        int maxIndex = 0;

        for(int i=0;i<array.length;i++) {
            if(array[i] > max){
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    private void detectImage(Mat image, int area){

        //Dectect AR
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds); // We can know the position of the Item in the picture

        // Streching the image
        //Get camera matrix
        Mat cameraMatrix = new Mat(3,3, CvType.CV_64F);
        cameraMatrix.put(3,3,api.getNavCamIntrinsics()[0]);
        Mat cameraCoefficients = new Mat(1,5,CvType.CV_64F);
        cameraCoefficients.put(0,0,api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

        //Undistort image
        Mat undistortImg = new Mat();
        Calib3d.fisheye_undistortImage(image, undistortImg, cameraMatrix, cameraCoefficients);

        //Pattern matching
        //Load template images
        Mat[] templates = new Mat[TEMPLATE_FILENAME.length];
        for(int i=0;i< TEMPLATE_FILENAME.length;i++) {
            try{
                //open the template image file in the Bitmap from the file name and convert to Mat
                InputStream inputStream = getAssets().open(TEMPLATE_FILENAME[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap,mat);

                //convert to grayscale
                Imgproc.cvtColor(mat,mat, Imgproc.COLOR_BGR2GRAY);

                //Assign to an array of templates
                templates[i] = mat;
                inputStream.close();

            }
            catch (IOException e) {
                e.printStackTrace();
            }
        }

        //Number of matches for each template
        int templateMatchCnt[] = new int[10];

        // Get the number of template matches
        for(int tempNum = 0; tempNum < templates.length; tempNum++) {
            // Number of matches
            int matchCnt = 0;
            // coordinates of the matched location
            List<org.opencv.core.Point> matches = new ArrayList<>();

            // loading template image and target image
            Mat template = templates[tempNum].clone();
            Mat targetImage = undistortImg.clone();

            //Pattern matching **px**
            int widthMin = 20;
            int widthMax = 100;
            int changeWidth = 5;
            int changeAngle = 45;

            for (int i = widthMin; i <= widthMax; i+= changeWidth) {
                for (int j=0; j<=360; j+= changeAngle) {
                    Mat resizedTemp = resizeImg(template, i);
                    Mat rotResizedTemp = rotImg(resizedTemp, j);

                    Mat result = new Mat();
                    Imgproc.matchTemplate(targetImage, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);

                    //Get coordinates with similarity greater than or equal to the treshold
                    double threshold = 0.8;
                    Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);
                    double maxVal = mmlr.maxVal;

                    if(maxVal >= threshold) {
                        // extract only results grater than or equal to the threshold
                        Mat thresholdedResult = new Mat();
                        Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                        //Get Match counts
                        for(int y=0; y< thresholdedResult.rows();y++) {
                            for(int x=0;x<thresholdedResult.cols();x++){
                                if(thresholdedResult.get(y,x)[0] > 0){
                                    matches.add(new org.opencv.core.Point(x,y));
                                }
                            }
                        }
                    }
                }
                // Avoidling detecting the same location multiple times
                List<org.opencv.core.Point> filteredMatches=removeDuplicates(matches);
                matchCnt += filteredMatches.size();

                //Number of matches for each template
                templateMatchCnt[tempNum] = matchCnt;
            }
        }

        //When you recognize items, let's set the type and number.
        int mostMatchTemplateNum = getMaxIndex(templateMatchCnt);
        api.setAreaInfo(area,TEMPLATE_NAME[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);

        return;
    }
}