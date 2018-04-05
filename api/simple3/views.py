from .serializers import JSONSerializer
from rest_framework import viewsets
from .models import JsonModel

class JSONViewSet(viewsets.ModelViewSet):
    """
    API endpoint that allows users to be viewed or edited.
    """
    queryset = JsonModel.objects.all().order_by('-id')[:50]
    serializer_class = JSONSerializer