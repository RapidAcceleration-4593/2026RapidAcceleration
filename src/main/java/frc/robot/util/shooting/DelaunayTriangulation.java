package frc.robot.util.shooting;

import java.util.ArrayList;
import java.util.List;
import org.tinfour.common.IIncrementalTin;
import org.tinfour.common.SimpleTriangle;
import org.tinfour.common.Vertex;
import org.tinfour.standard.IncrementalTin;
import org.tinfour.utils.TriangleCollector;

/**
 * Delaunay Triangulation using the Tinfour library.
 *
 * <p><b>Tinfour Maven dependency:</b>
 *
 * <pre>{@code
 * <dependency>
 *   <groupId>org.tinfour</groupId>
 *   <artifactId>Tinfour-Core</artifactId>
 *   <version>2.1.7</version>
 * </dependency>
 * }</pre>
 *
 * <p><b>Gradle:</b>
 *
 * <pre>{@code
 * implementation 'org.tinfour:Tinfour-Core:2.1.7'
 * }</pre>
 *
 * <p><b>Usage:</b>
 *
 * <pre>{@code
 * double[][] points = { {0,0}, {1,0}, {0,1}, {1,1}, {0.5, 0.5} };
 * int[][] triangles = DelaunayTriangulation.triangulate(points);
 * // Each row in triangles is [i, j, k] — indices into the original points array.
 * }</pre>
 */
public class DelaunayTriangulation {

    /**
     * Performs a Delaunay triangulation on the provided 2D points.
     *
     * @param points a double[][] where each row is {x, y}
     * @return an int[][] where each row is {i, j, k} — zero-based indices into {@code points} representing one
     *     triangle; never null, may be empty if fewer than 3 non-collinear points are supplied
     * @throws IllegalArgumentException if {@code points} is null, or if any row has fewer than 2 elements
     */
    public static int[][] triangulate(double[][] points) {
        if (points == null) {
            throw new IllegalArgumentException("points array must not be null");
        }
        if (points.length < 3) {
            return new int[0][];
        }

        // Build Tinfour Vertex objects.
        // The 4th constructor argument is an integer "index" that Tinfour stores
        // on each Vertex so we can map back to the original array row later.
        List<Vertex> vertices = new ArrayList<>(points.length);
        for (int i = 0; i < points.length; i++) {
            if (points[i] == null || points[i].length < 2) {
                throw new IllegalArgumentException("points[" + i + "] must have at least 2 elements");
            }
            // Vertex(x, y, z, index) — z is unused; we set it to 0.
            vertices.add(new Vertex(points[i][0], points[i][1], 0.0, i));
        }

        // Build the incremental TIN (Triangulated Irregular Network)
        IIncrementalTin tin = new IncrementalTin();
        tin.add(vertices, null); // null = no progress monitor

        // Collect every triangle in the TIN
        List<SimpleTriangle> simpleTriangles = new ArrayList<>();
        TriangleCollector.visitSimpleTriangles(tin, simpleTriangles::add);

        // Convert SimpleTriangle list → int[][] using the stored vertex indices
        int[][] result = new int[simpleTriangles.size()][3];
        for (int t = 0; t < simpleTriangles.size(); t++) {
            SimpleTriangle tri = simpleTriangles.get(t);
            result[t][0] = tri.getVertexA().getIndex();
            result[t][1] = tri.getVertexB().getIndex();
            result[t][2] = tri.getVertexC().getIndex();
        }
        return result;
    }
}
