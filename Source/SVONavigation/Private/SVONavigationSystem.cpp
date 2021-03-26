#include "SVONavigationSystem.h"

void USVONavigationSystem::OnNavigationVolumeAdded( const ASVONavigationVolume & volume )
{
}

void USVONavigationSystem::OnNavigationVolumeRemoved( const ASVONavigationVolume & volume )
{
}

void USVONavigationSystem::OnNavigationVolumeUpdated( const ASVONavigationVolume & volume )
{
    FSVONavigationBoundsUpdateRequest UpdateRequest;
    UpdateRequest.NavBounds.UniqueID = volume.GetUniqueID();
    //UpdateRequest.NavBounds.AreaBox = NavVolume->GetComponentsBoundingBox( true );
    UpdateRequest.NavBounds.Level = volume.GetLevel();
    //UpdateRequest.NavBounds.SupportedAgents = NavVolume->SupportedAgents;

    UpdateRequest.UpdateRequest = FSVONavigationBoundsUpdateRequest::Type::Updated;
    AddNavigationVolumeUpdateRequest( UpdateRequest );
}

void USVONavigationSystem::AddNavigationVolumeUpdateRequest( const FSVONavigationBoundsUpdateRequest & update_request )
{
    const auto existing_id = PendingNavigationVolumeUpdateRequests.IndexOfByPredicate( [ & ]( const FSVONavigationBoundsUpdateRequest & Element ) {
        return update_request.NavBounds.UniqueID == Element.NavBounds.UniqueID;
    } );

    if ( existing_id != INDEX_NONE )
    {
        // catch the case where the bounds was removed and immediately re-added with the same bounds as before
        // in that case, we can cancel any update at all
        auto it_can_cancel_update = false;
        if ( PendingNavigationVolumeUpdateRequests[ existing_id ].UpdateRequest == FSVONavigationBoundsUpdateRequest::Type::Removed && update_request.UpdateRequest == FSVONavigationBoundsUpdateRequest::Type::Added )
        {
            for ( TSet< FSVONavigationBounds >::TConstIterator It( RegisteredNavigationBounds ); It; ++It )
            {
                if ( *It == update_request.NavBounds )
                {
                    it_can_cancel_update = true;
                    break;
                }
            }
        }
        if ( it_can_cancel_update )
        {
            PendingNavigationVolumeUpdateRequests.RemoveAt( existing_id );
        }
        else
        {
            // Overwrite any previous updates
            PendingNavigationVolumeUpdateRequests[ existing_id ] = update_request;
        }
    }
    else
    {
        PendingNavigationVolumeUpdateRequests.Add( update_request );
    }
}
