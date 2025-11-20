#include "SVONavigationDataGenerator.h"
#include "SVONavigationData.h"
#include "NavigationSystem.h"

FSVOVolumeNavigationDataGenerator::FSVOVolumeNavigationDataGenerator( FSVONavigationDataGenerator & navigation_data_generator, const FBox & volume_bounds ) :
	ParentGenerator( navigation_data_generator ),
	BoundsNavigationData(),
	VolumeBounds( volume_bounds )
{
	NavDataConfig = navigation_data_generator.GetOwner()->GetConfig();
}

bool FSVOVolumeNavigationDataGenerator::DoWork()
{
	FSVOVolumeNavigationDataGenerationSettings generation_settings;
	generation_settings.GenerationSettings = ParentGenerator.GetGenerationSettings();
	generation_settings.World = ParentGenerator.GetWorld();
	generation_settings.VoxelExtent = NavDataConfig.AgentRadius * 2.0f;

	BoundsNavigationData.GenerateNavigationData( VolumeBounds, generation_settings );

	return true;
}

FSVONavigationDataGenerator::FSVONavigationDataGenerator( ASVONavigationData & navigation_data ) :
	NavigationData( navigation_data ),
	MaximumGeneratorTaskCount( 2 ),
	IsInitialized( false )
{
}

void FSVONavigationDataGenerator::Init()
{
	GenerationSettings = NavigationData.GenerationSettings;

	UpdateNavigationBounds();

	///** setup maximum number of active tile generator*/
	const int32 worker_threads_count = FTaskGraphInterface::Get().GetNumWorkerThreads();
	MaximumGeneratorTaskCount = FMath::Min( FMath::Max( worker_threads_count * 2, 1 ), NavigationData.MaxSimultaneousBoxGenerationJobsCount );
	UE_LOG( LogNavigation, Log, TEXT( "Using max of %d workers to build SVO navigation." ), MaximumGeneratorTaskCount );
}