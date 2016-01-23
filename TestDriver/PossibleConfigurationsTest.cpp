#include "stdafx.h"
#using<system.dll>
using namespace System;
using namespace System::Text;
using namespace System::Collections::Generic;
using namespace	Microsoft::VisualStudio::TestTools::UnitTesting;
using namespace System::IO;

#include <iostream>
#include "../CFObject/PossibleConfigurations.h"

using namespace std;
namespace TestDriver
{
	[TestClass]
	public ref class PossibleConfigurationsTest
	{
	public: 
		[TestMethod]
		void TestConstrutor()
		{
			Transformation2D trans(0,0,0);
			Transformation2D arrayoftrans[1];
			arrayoftrans[0] = trans;
			PossibleConfigurations pC(arrayoftrans, 1);

			Cluster c1(1);
			c1.setX(0, 0.0f);
			c1.setY(0, 1.207813763297230f);
			c1.setZ(0, 2.077142003943222f);

			pC.findBestFit(c1);
			

		}

		[TestMethod]
		void TestFitting()
		{
			int nOfFitting;
			float R[9];
			Transformation2D* arrayoftrans;

			String^ fileName = "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleFitting.bin";
			try
			{
				FileStream^ fs = gcnew FileStream(fileName, FileMode::Open);
				BinaryReader^ br = gcnew BinaryReader(fs);

				nOfFitting = br->ReadInt32();
				arrayoftrans = new Transformation2D[nOfFitting];
				float* RBuffer = new float[nOfFitting*9];
				for(int i=0; i<nOfFitting*9; i++)
				{
					RBuffer[i] = br->ReadSingle();
				}

				for(int i=0; i<nOfFitting; i++)
				{
					arrayoftrans[i].setR(0, RBuffer[i*9+0]);
					arrayoftrans[i].setR(1, RBuffer[i*9+1]);
					arrayoftrans[i].setR(2, RBuffer[i*9+3]);
					arrayoftrans[i].setR(3, RBuffer[i*9+4]);					
				}
				delete RBuffer;

				for(int i=0; i<nOfFitting; i++)
				{
					arrayoftrans[i].setv(0, br->ReadSingle());
				}

				for(int i=0; i<nOfFitting; i++)
				{
					arrayoftrans[i].setv(1, br->ReadSingle());
				}

				fs->Close( );
			}
			catch (Exception^ e)
			{
				if (dynamic_cast<FileNotFoundException^>(e))
					Console::WriteLine("File '{0}' not found", fileName);
				else
					Console::WriteLine("Exception: ({0})", e);
				
			}
			PossibleConfigurations pC(arrayoftrans, nOfFitting);

			fileName = "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleCloud.bin";
			Cluster c1;
			try
			{
				FileStream^ fs = gcnew FileStream(fileName, FileMode::Open);
				BinaryReader^ br = gcnew BinaryReader(fs);

				int nOfPoints = br->ReadInt32();
				Cluster c(nOfPoints);
				arrayoftrans = new Transformation2D[nOfPoints];
				

				for(int i=0; i<nOfPoints; i++)
				{
					c.setX(i, br->ReadSingle());
				}
				for(int i=0; i<nOfPoints; i++)
				{
					c.setY(i, br->ReadSingle());
				}
				for(int i=0; i<nOfPoints; i++)
				{
					c.setZ(i, br->ReadSingle());
				}

				fs->Close( );
				c1 = c;

			}
			catch (Exception^ e)
			{
				if (dynamic_cast<FileNotFoundException^>(e))
					Console::WriteLine("File '{0}' not found", fileName);
				else
					Console::WriteLine("Exception: ({0})", e);
				
			}

			fileName = "C:\\ra\\GitHub\\CostFunction\\TestData\\sampleFittingProbs.bin";			
			double* probs;
			try
			{
				FileStream^ fs = gcnew FileStream(fileName, FileMode::Open);
				BinaryReader^ br = gcnew BinaryReader(fs);

				int nOfPoints = br->ReadInt32();
				probs = new double[nOfPoints];
			
				for(int i=0; i<nOfPoints; i++)
				{
					probs[i] = br->ReadDouble();
				}
				

				fs->Close( );				
			}
			catch (Exception^ e)
			{
				if (dynamic_cast<FileNotFoundException^>(e))
					Console::WriteLine("File '{0}' not found", fileName);
				else
					Console::WriteLine("Exception: ({0})", e);
				
			}

			
			pC.findBestFit(c1);
			for(int i=0; i<pC.getSize(); i++)
			{
				Assert::AreEqual(probs[i], pC.getProb()[i], 1e-5);
			}
			Assert::AreEqual(1.0, pC.getMaxProb(), 1e-5);
			Transformation2D trans2D = pC.getBestTransformation();
			Assert::AreEqual(-0.08293f, trans2D.getv()[0], 1e-5f);

	
			delete[] arrayoftrans;

		}


	};
}
