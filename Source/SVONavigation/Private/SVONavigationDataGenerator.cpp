#include "SVONavigationDataGenerator.h"


#include "NavigationSystem.h"
#include "SVONavigationData.h"
#include "NavMesh/RecastNavMesh.h"

namespace
{
    bool IsStaticNavigationData( const ASVONavigationData & navigation_data )
    {
        return ( navigation_data.GetWorld()->IsGameWorld() && navigation_data.GetRuntimeGenerationMode() != ERuntimeGenerationType::Dynamic );
    }
}

FSVONavigationDataGenerator::FSVONavigationDataGenerator( ASVONavigationData & navigation_data ) :
    NavigationData( navigation_data ),
    IsInitialized( false )
{
}

FSVONavigationDataGenerator::~FSVONavigationDataGenerator()
{
}

void FSVONavigationDataGenerator::Init()
{
    BuildConfig = NavigationData.Config;

    UpdateNavigationBounds();

    /** setup maximum number of active tile generator*/
    const int32 NumberOfWorkerThreads = FTaskGraphInterface::Get().GetNumWorkerThreads();
    MaxBoxGeneratorTasks = FMath::Min( FMath::Max( NumberOfWorkerThreads * 2, 1 ), NavigationData.MaxSimultaneousBoxGenerationJobsCount );
    UE_LOG( LogNavigation, Log, TEXT( "Using max of %d workers to build SVO navigation." ), MaxBoxGeneratorTasks );

    IsInitialized = true;

    // recreate navmesh if no data was loaded, or when loaded data doesn't match current grid layout
    bool must_create_navigation_data = true;
    const bool is_static_navigation_data = IsStaticNavigationData( NavigationData );

    if ( is_static_navigation_data )
    {
        must_create_navigation_data = false;
    }
    else
    {
        // :TODO: ?
    };

    if ( must_create_navigation_data )
    {
        // :TODO:
        //ConstructTiledNavMesh();
        MarkNavBoundsDirty();
    }
}

bool FSVONavigationDataGenerator::MarkNavBoundsDirty()
{
    // if rebuilding all no point in keeping "old" invalidated areas
    TArray< FNavigationDirtyArea > DirtyAreas;
    for ( FBox AreaBounds : InclusionBounds )
    {
        FNavigationDirtyArea DirtyArea( AreaBounds, ENavigationDirtyFlag::All | ENavigationDirtyFlag::NavigationBounds );
        DirtyAreas.Add( DirtyArea );
    }

    if ( DirtyAreas.Num() )
    {
        // :TODO:
        // MarkDirtyTiles( DirtyAreas );
        return true;
    }
    return false;
}

void FSVONavigationDataGenerator::UpdateNavigationBounds()
{
    if ( const UNavigationSystemV1 * navigation_system = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( GetWorld() ) )
    {
        if ( navigation_system->ShouldGenerateNavigationEverywhere() == false )
        {
            FBox BoundsSum( ForceInit );
            
            TArray< FBox > SupportedBounds;
            navigation_system->GetNavigationBoundsForNavData( NavigationData, SupportedBounds );
            InclusionBounds.Reset( SupportedBounds.Num() );

            for ( const FBox & Box : SupportedBounds )
            {
                InclusionBounds.Add( Box );
                BoundsSum += Box;
            }
            
            TotalNavBounds = BoundsSum;
        }
        else
        {
            InclusionBounds.Reset( 1 );
            TotalNavBounds = navigation_system->GetWorldBounds();
            if ( !TotalNavBounds.IsValid )
            {
                InclusionBounds.Add( TotalNavBounds );
            }
        }
    }
    else
    {
        TotalNavBounds = FBox( ForceInit );
    }
}
