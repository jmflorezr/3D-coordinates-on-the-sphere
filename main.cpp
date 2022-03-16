#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkPNGReader.h>
#include <vtkImageDataGeometryFilter.h>
#include <vtkWarpScalar.h>
#include <vtkLookupTable.h>
#include <vtkImageData.h>
#include <vtkInteractorStyle.h>
#include <vtkDelaunay2D.h>
#include <vtkPolyDataWriter.h>
#include <vtkPolyDataReader.h>
#include <vtkCallbackCommand.h>
#include <vtkImageAccumulate.h>
#include <vtkAbstractPicker.h>
#include <vtkRendererCollection.h>
#include <vtkPointPicker.h>
#include "iostream"
#include "string"
#include <vtkCleanPolyData.h>
#include <vtkDijkstraGraphGeodesicPath.h>

#define vtkSPtr vtkSmartPointer
#define vtkSPtrNew(Var, Type) vtkSPtr<Type> Var = vtkSPtr<Type>::New();


using namespace std;

/**
 * Reads a PNG grayscale image and generates and Delaunay triangular mesh vtkFile
 * The vtk PolyData file is persisted for later use
 * @param imgName path and name of the pngImage e.g ../Img/myMap.png , to generate mesh from
 * @param meshName path and name of the pngImage e.g ../Img/myMap.vtk, to persist mesh into
 * @param step real height of the height map for normalization
 */
void generateDelaunayMesh(string imgName = "", string meshName = "", double maxHeight = 0) {

	if (imgName.empty() || meshName.empty()) {
		return;
	}
	else {
		//std::cout << "Entrando... " << std::endl;
		/* Reads the image with the grey scale height values */
		vtkSPtrNew(reader, vtkPNGReader);
		reader->SetFileName(imgName.c_str());
		reader->Update();
		double lo = reader->GetOutput()->GetScalarRange()[0]; //Equivalente a vtkImageAccumulate utilizado en el taller 2 para extraer el punto máximo y punto  mínimo
		double hi = reader->GetOutput()->GetScalarRange()[1];

		std::cout << "lo... " << lo << std::endl;
		std::cout << "hi... " << hi << std::endl;

		//Extraer geometría
		vtkSPtrNew(surface, vtkImageDataGeometryFilter);
		surface->SetInputConnection(reader->GetOutputPort());
		surface->Update();

		// Triangulate the grid points
		vtkNew<vtkDelaunay2D> delaunay;
		delaunay->SetInputConnection(surface->GetOutputPort());
		delaunay->Update();


		vtkSPtrNew(warpGemoetry, vtkWarpScalar);
		warpGemoetry->SetInputConnection(delaunay->GetOutputPort());
		double step = maxHeight / hi;  // divide mountain height by image gray value (normalize)
		warpGemoetry->SetScaleFactor(step);
		warpGemoetry->UseNormalOn();
		warpGemoetry->SetNormal(0, 0, 1);

		vtkSPtrNew(writer, vtkPolyDataWriter);
		writer->SetInputConnection(warpGemoetry->GetOutputPort());
		writer->SetFileName(meshName.c_str());
		writer->Write();
	}
}

double vertexID = 0;
bool aux = true;
/*
* Custom function for key event
*/
void keyboardCallbackFunction(vtkObject* caller, unsigned long eventId, void* clientData, void* callData) {
	vtkSPtrNew(interactor, vtkRenderWindowInteractor);
	interactor = static_cast<vtkRenderWindowInteractor*>(caller);
	string keyPressed = interactor->GetKeySym();
	if (keyPressed == "p" || keyPressed == "P") {
		
		int x = interactor->GetEventPosition()[0];
		int y = interactor->GetEventPosition()[1];

		
		std::cout << "Vertex ID pointed: ";

		vtkSPtrNew(renderer, vtkRenderer);
		renderer = interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();

		vtkSPtrNew(pointPicker, vtkPointPicker);
		pointPicker->Pick(x,y,0,renderer);
		vertexID = pointPicker->GetPointId ();
		if(pointPicker->GetPointId()>=0){
			std::cout << vertexID << std::endl;
		}else{
			std::cout << "Not Found" << std::endl;
		}


	}
}

vtkSPtrNew(reader, vtkPolyDataReader);
vtkSPtrNew(renderer, vtkRenderer);
vtkSPtrNew(renderWindow, vtkRenderWindow);
vtkNew<vtkDijkstraGraphGeodesicPath> dijkstra;
void keyboardCallbackFunctionDijkstra(vtkObject* caller, unsigned long eventId, void* clientData, void* callData) {
	vtkSPtrNew(interactor, vtkRenderWindowInteractor);
	interactor = static_cast<vtkRenderWindowInteractor*>(caller);
	string keyPressed = interactor->GetKeySym();
	if (keyPressed == "p" || keyPressed == "P") {
		
		dijkstra->SetInputConnection(reader->GetOutputPort());

		if(aux == true){
			dijkstra->SetStartVertex(vertexID);	
			aux = false;
			std::cout<<"Start: "<<vertexID<<std::endl;
		}
		else{
			dijkstra->SetEndVertex(vertexID);
			aux = true;
			std::cout<<"End: "<<vertexID<<std::endl;

			dijkstra->Update();

			vtkNew<vtkPolyDataMapper> pathMapper;
			pathMapper->SetInputConnection(dijkstra->GetOutputPort());

			vtkSPtrNew(mapActor2, vtkActor);
			vtkSPtrNew(colors, vtkNamedColors);
			mapActor2->SetMapper(pathMapper);
			mapActor2->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());
			mapActor2->GetProperty()->SetLineWidth(4);
			renderer->AddActor(mapActor2);
			renderWindow->Render();
		}

	}
}




int main() {

	generateDelaunayMesh ("../Img/Mountain.png","../Img/Mountain.vtk",100.0);

	
	
//	vtkSPtrNew(reader, vtkPolyDataReader);
	reader->SetFileName("../Img/Mountain.vtk");
	reader->Update();
	double lo = reader->GetOutput()->GetScalarRange()[0];
	double hi = reader->GetOutput()->GetScalarRange()[1];

	// Map scalar (height) values into color (RGBA)
	vtkSPtrNew(lut, vtkLookupTable);
	lut->SetHueRange(0.65, 0);
	lut->SetSaturationRange(0.9, 0);
	lut->SetValueRange(0.25, 1.0);

	// Maps polygonal data to graphics primitives for later GPU rendering
	vtkSPtrNew(mapMapper, vtkPolyDataMapper);
	mapMapper->SetInputConnection(reader->GetOutputPort());
	mapMapper->SetScalarRange(lo, hi);
	mapMapper->SetLookupTable(lut);

	vtkSPtrNew(mapActor, vtkActor);
	mapActor->SetMapper(mapMapper);

	vtkSPtrNew(colors, vtkNamedColors);
	mapActor->GetProperty()->SetSpecular(.3);
	mapActor->GetProperty()->SetSpecularPower(3);
	mapActor->GetProperty()->SetRepresentationToWireframe();

	/*vtkNew<vtkDijkstraGraphGeodesicPath> dijkstra;
	dijkstra->SetInputConnection(reader->GetOutputPort());
	if(aux == true){
		dijkstra->SetStartVertex(vertexID);	
		aux = false;
		std::cout<<"Start: "<<vertexID<<std::endl;
	}
	else{
		dijkstra->SetEndVertex(vertexID);
		aux = true;
		std::cout<<"End: "<<vertexID<<std::endl;
	}
	dijkstra->Update();

	vtkNew<vtkPolyDataMapper> pathMapper;
	pathMapper->SetInputConnection(dijkstra->GetOutputPort());

	vtkSPtrNew(mapActor2, vtkActor);
	mapActor2->SetMapper(pathMapper);
	mapActor2->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());

	vtkSPtrNew(renderer, vtkRenderer);*/
	renderer->AddActor(mapActor);
//	renderer->AddActor(mapActor2);	
	renderer->SetBackground(0.5, 0.6, 0.7);

	vtkSPtrNew(keyboardCallback, vtkCallbackCommand);
	keyboardCallback->SetCallback(keyboardCallbackFunction);

	vtkSPtrNew(keyboardCallbackDijkstra, vtkCallbackCommand);
	keyboardCallbackDijkstra->SetCallback(keyboardCallbackFunctionDijkstra);

	//vtkSPtrNew(renderWindow, vtkRenderWindow);
	renderWindow->AddRenderer(renderer);

	vtkSPtrNew(interactor, vtkRenderWindowInteractor);
	interactor->SetRenderWindow(renderWindow);
	interactor->AddObserver(vtkCommand::KeyPressEvent, keyboardCallback);
	interactor->AddObserver(vtkCommand::KeyPressEvent, keyboardCallbackDijkstra);

	renderer->ResetCamera();
	renderWindow->Render();
	interactor->Start();

	return 0;
}
