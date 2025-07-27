from django.urls import path

# from gwanje import views
from . import views

app_name = 'gwanje' 

urlpatterns = [
    path('cam', views.gwanje_cam , name='gwanje_cam' ), 

    path('video_feed/', views.video_feed, name='video_feed'),
    # path('markers/', views.markers_api, name='markers_api'),



]




