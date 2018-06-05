#include "pcl_viewer_custom.hh"

#include <vtkVersion.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkWindowToImageFilter.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkObjectFactory.h>


#if VTK_RENDERING_BACKEND_OPENGL_VERSION < 2
#include <pcl/visualization/vtk/vtkVertexBufferObjectMapper.h>
#endif


// Standard VTK macro for *New ()
vtkStandardNewMacro(PCLInteractorCustom);

void PCLInteractorCustom::OnKeyDown () 
{
	using namespace pcl::visualization;

	if (!init_)
	{
		pcl::console::print_error ("[PCLVisualizerInteractorStyle] Interactor style not initialized. Please call Initialize () before continuing.\n");
		return;
	}

	if (!rens_)
	{
		pcl::console::print_error ("[PCLVisualizerInteractorStyle] No renderer collection given! Use SetRendererCollection () before continuing.\n");
		return;
	}

	FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

	if (wif_->GetInput () == NULL)
	{
		wif_->SetInput (Interactor->GetRenderWindow ());
		wif_->Modified ();
		// snapshot_writer_->Modified ();
	}

	// Save the initial windows width/height
	if (win_height_ == -1 || win_width_ == -1)
	{
		int *win_size = Interactor->GetRenderWindow ()->GetSize ();
		win_height_ = win_size[0];
		win_width_  = win_size[1];
	}

	// // Get the status of special keys (Cltr+Alt+Shift)
	// bool shift = Interactor->GetShiftKey   ();
	// bool ctrl  = Interactor->GetControlKey ();
	// bool alt   = Interactor->GetAltKey ();

	// bool keymod = false;
	// switch (modifier_)
	// {
	// 	case INTERACTOR_KB_MOD_ALT:
	// 	{
	// 	  keymod = alt;
	// 	  break;
	// 	}
	// 	case INTERACTOR_KB_MOD_CTRL:
	// 	{
	// 	  keymod = ctrl;
	// 	  break;
	// 	}
	// 	case INTERACTOR_KB_MOD_SHIFT:
	// 	{
	// 	  keymod = shift;
	// 	  break;
	// 	}
	// }

	std::string key (Interactor->GetKeySym ());
	if (key.find ("XF86ZoomIn") != std::string::npos)
		zoomIn ();
	else if (key.find ("XF86ZoomOut") != std::string::npos)
		zoomOut ();

	switch (Interactor->GetKeyCode ())
	{
		case 27: // ESC
		{
		  Interactor->ExitCallback ();
		  return;
		}
		case 'q': case 'Q': case 'e': case 'E':
		{
			break;
		}
		default:
		{
		  Superclass::OnKeyDown ();
		  break;
		}
	}

	KeyboardEvent event (true, Interactor->GetKeySym (), Interactor->GetKeyCode (), Interactor->GetAltKey (), Interactor->GetControlKey (), Interactor->GetShiftKey ());
	keyboard_signal_ (event);

	rens_->Render ();
	Interactor->Render ();
}

void PCLInteractorCustom::OnChar ()
{
	// Make sure we ignore the same events we handle in OnKeyDown to avoid calling things twice
	FindPokedRenderer (Interactor->GetEventPosition ()[0], Interactor->GetEventPosition ()[1]);

	std::string key (Interactor->GetKeySym ());
	if (key.find ("XF86ZoomIn") != std::string::npos)
		zoomIn ();
	else if (key.find ("XF86ZoomOut") != std::string::npos)
		zoomOut ();

	switch (Interactor->GetKeyCode ())
	{	
		case 'q': case 'Q': case 'e': case 'E':
		{
			break;
		}
	    default:
	    {
			Superclass::OnChar ();
			break;
	    }
	}
}
