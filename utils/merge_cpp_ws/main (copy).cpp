#include <KrisLibrary/geometry/TSDFReconstruction.h>
#include <KrisLibrary/meshing/IO.h>
#include <Klampt/Modeling/World.h>
#include <KrisLibrary/GLdraw/GLUTNavigationProgram.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLRenderToImage.h>
#include <GL/gl.h>
using namespace Geometry;
using namespace Meshing;
using namespace std;
using namespace GLDraw;
int main(int argc,char** argv)
{
	//load first point cloud
	PointCloud3D pc;
	const char* fn1 = "../experiment_data/processed/objectScan_0.pcd";
	//char* fn1 = "../pcd_data/objectScan_0.pcd";
	bool res;
	res = pc.LoadPCL(fn1);
	bool hasColor;
	hasColor = pc.HasColor();
	if (res) cout << "----- Load first scan success -----" << hasColor << "\n";
	

	//initialize the identity transform
	double Rotation[3][3]= {1.0,0,0,0,1.0,0,0,0,1.0};//{{1.0,0,0},{0,1.0,0},{0,0,1.0}};
	Matrix3 R(Rotation);
	Vector3 translation1(0,0,0);
	Vector3 translation2(0,0.0005,0);
	RigidTransform Tcamera1(R,translation1);//identity transformation
	RigidTransform Tcamera2(R,translation2);
		
	//Initialize TSDF and fuse first pcd
	/* This way is from the TSDF test.cpp, but fuse gives segfault..
	shared_ptr<SparseTSDFReconstruction> reconstruction;
	reconstruction->Fuse(Tcamera1,pc);
	TriMesh mesh;
	reconstruction -> ExtractMesh(mesh);
	*/
	Vector3 cellSize = {0.001,0.001,0.006};
	SparseTSDFReconstruction reconstruction(cellSize);
	reconstruction.Fuse(Tcamera1,pc);

	// really bad code.... but I'm not sure how to concatenate char, and whether
	// pc.loadPCL will remove the existing pcl...
	const char* fn2 = "../experiment_data/processed/objectScan_1.pcd";
	const char* fn3 = "../experiment_data/processed/objectScan_2.pcd";
	const char* fn4 = "../experiment_data/processed/objectScan_3.pcd";
	const char* fn5 = "../experiment_data/processed/objectScan_4.pcd";
	PointCloud3D pc1;
	PointCloud3D pc2;
	PointCloud3D pc3;
	PointCloud3D pc4;
	pc1.LoadPCL(fn2);
	pc2.LoadPCL(fn3);
	pc3.LoadPCL(fn4);
	pc4.LoadPCL(fn5);
	reconstruction.Fuse(Tcamera1,pc1);
	reconstruction.Fuse(Tcamera1,pc2);
	reconstruction.Fuse(Tcamera1,pc3);
	reconstruction.Fuse(Tcamera1,pc4);

	//Extract a colored mesh
	GeometryAppearance app;
	TriMesh mesh;
	reconstruction.ExtractMesh(mesh,app);
	//app.SetColor(1.0f,0.2f,0.2f,0.5f);
	const char* fn0 = "../experiment_data/TSDF_result.ply"; // ply format seems to be the only
										// cross section between Assimp and Open3D....
	cout << "flag1\n" ;
	//res = Export(fn2,mesh,app);
	res = SaveAssimp(fn0,mesh,app);
	if (res) cout << "----- Extracted mesh exported -----" << "\n";
	
	// simply saving the mesh by: mesh.Save(fn2); does not work... this saves 
	// it as a TriMesh file....
	return 0;


	/*
	// load and fuse multiple pcs

	//reconstruction -> Fuse(Tcamera2,pc)
	*/
}